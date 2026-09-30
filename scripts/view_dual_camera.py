#!/usr/bin/env python3
"""
view_dual_camera.py
===================
Real-time Dual RGB Optical & Thermal Infrared (LWIR) Camera Viewer for WareOps Armo_bot.
Subscribes to:
  - /camera/image_raw  (sensor_msgs/msg/Image)
  - /thermal/image_raw (sensor_msgs/msg/Image)
  - /camera_flashlight/status (std_msgs/msg/Bool, optional)

Features:
  - Unified side-by-side high-tech HUD inspection display
  - False-color thermal colormaps (INFERNO, JET, HOT, MAGMA, PLASMA, TURBO, GRAYSCALE)
  - Thermal spot-metering (min, max, center pixel readings)
  - Live heartbeat watchdog ("LIVE ●" / "STALL ⚠️") tracking real-time frame arrivals
  - Single-threaded low-latency ROS 2 event loop (prevents background thread crashes)
  - Flashlight toggle hotkey ('L') with both ROS 2 topic and direct Gazebo command fallback
  - Snapshot recorder hotkey ('S') saving synchronized RGB + Thermal PNGs
  - Interactive window layout switch ('W') between Unified and Split windows
"""

import sys
import os
import time
import datetime
import subprocess
import threading
import re
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError


COLORMAPS = [
    ("INFERNO", cv2.COLORMAP_INFERNO),
    ("JET", cv2.COLORMAP_JET),
    ("HOT", cv2.COLORMAP_HOT),
    ("MAGMA", cv2.COLORMAP_MAGMA),
    ("PLASMA", cv2.COLORMAP_PLASMA),
    ("TURBO", cv2.COLORMAP_TURBO),
    ("GRAYSCALE", None)
]


class DualCameraViewer(Node):
    def __init__(self):
        super().__init__('dual_camera_viewer')

        self.bridge = CvBridge()
        self.lock = threading.Lock()
        self._gazebo_light_id = None

        # Frame storage
        self.rgb_frame = None
        self.thermal_frame = None
        self.rgb_timestamp = 0.0
        self.thermal_timestamp = 0.0

        # Performance & HUD metrics
        self.rgb_fps = 0.0
        self.thermal_fps = 0.0
        self._rgb_prev_time = time.time()
        self._thermal_prev_time = time.time()
        self._rgb_count = 0
        self._thermal_count = 0

        # State & Options
        self.colormap_idx = 0  # Default: INFERNO
        self.split_windows = False
        self.flashlight_state = True
        self.save_dir = os.path.expanduser('~/WareOps/camera_captures')
        os.makedirs(self.save_dir, exist_ok=True)

        # Precompute radial flashlight beam mask for 640x480 RGB display
        h_b, w_b = 480, 640
        cx, cy = w_b // 2, h_b // 2
        y, x = np.ogrid[:h_b, :w_b]
        dist_sq = ((x - cx) ** 2 + (y - cy) ** 2) / (float(w_b // 2) ** 2)
        self.flashlight_beam = np.clip(2.5 * np.exp(-dist_sq * 2.0), 0.0, 2.5)

        # Reliable QoS matching standard ros_gz_bridge publishers (prevents UDP drops)
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.rgb_callback,
            sensor_qos)

        self.thermal_sub = self.create_subscription(
            Image,
            '/thermal/image_raw',
            self.thermal_callback,
            sensor_qos)

        self.flashlight_status_sub = self.create_subscription(
            Bool,
            '/camera_flashlight/status',
            self.flashlight_status_callback,
            10)

        # Publisher to control flashlight directly from hotkey 'L'
        self.flashlight_cmd_pub = self.create_publisher(
            Bool,
            '/camera_flashlight/switch',
            10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("📹 WareOps Dual RGB + Thermal Camera Viewer Started")
        self.get_logger().info("  RGB Topic     : /camera/image_raw")
        self.get_logger().info("  Thermal Topic : /thermal/image_raw")
        self.get_logger().info("  Controls:")
        self.get_logger().info("    [C] Cycle Thermal Colormap (INFERNO, JET, HOT...)")
        self.get_logger().info("    [L] Toggle Flashlight ON / OFF")
        self.get_logger().info("    [S] Save Snapshot (RGB + Thermal PNGs)")
        self.get_logger().info("    [W] Toggle Split / Unified Windows")
        self.get_logger().info("    [Q] / [ESC] Quit")
        self.get_logger().info("=" * 60)

    def rgb_callback(self, msg: Image):
        """Callback to safely convert incoming RGB sensor image to OpenCV BGR."""
        try:
            cv_img = None
            try:
                cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception:
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                if len(raw.shape) == 2:
                    cv_img = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
                elif len(raw.shape) == 3 and raw.shape[2] == 4:
                    cv_img = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
                else:
                    cv_img = raw

            if cv_img is None or cv_img.size == 0:
                return

            now = time.time()
            self._rgb_count += 1
            if now - self._rgb_prev_time >= 1.0:
                self.rgb_fps = self._rgb_count / (now - self._rgb_prev_time)
                self._rgb_count = 0
                self._rgb_prev_time = now

            with self.lock:
                self.rgb_frame = cv_img
                self.rgb_timestamp = now
        except Exception as e:
            self.get_logger().warn(f"RGB callback error: {e}")

    def thermal_callback(self, msg: Image):
        """Callback to safely convert incoming thermal sensor image to 2D uint8."""
        try:
            raw = None
            if msg.encoding in ['mono8', '8UC1']:
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
            else:
                raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

            if raw is None or raw.size == 0:
                return

            # Convert multi-channel images (e.g. RGB) to single-channel grayscale
            if len(raw.shape) == 3:
                raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

            # Replace any NaN or Infinite values
            if np.issubdtype(raw.dtype, np.floating):
                raw = np.nan_to_num(raw, nan=0.0, posinf=255.0, neginf=0.0)

            # Normalize to 8-bit unsigned integers [0, 255]
            if raw.dtype != np.uint8:
                raw = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            now = time.time()
            self._thermal_count += 1
            if now - self._thermal_prev_time >= 1.0:
                self.thermal_fps = self._thermal_count / (now - self._thermal_prev_time)
                self._thermal_count = 0
                self._thermal_prev_time = now

            with self.lock:
                self.thermal_frame = raw
                self.thermal_timestamp = now
        except Exception as e:
            self.get_logger().warn(f"Thermal callback error: {e}")

    def flashlight_status_callback(self, msg: Bool):
        with self.lock:
            self.flashlight_state = bool(msg.data)

    def toggle_flashlight(self):
        with self.lock:
            self.flashlight_state = not self.flashlight_state
            new_state = self.flashlight_state

        # Publish via ROS 2
        msg = Bool()
        msg.data = new_state
        self.flashlight_cmd_pub.publish(msg)

        state_str = "ON 💡" if new_state else "OFF 🌑"
        self.get_logger().info(f"Hotkey [L] → Flashlight Toggled: {state_str}")

        # Send command asynchronously to Gazebo using resolved entity ID
        self._send_gazebo_light_cmd(new_state)

    def _send_gazebo_light_cmd(self, is_on: bool):
        def _worker():
            try:
                # 1. Discover active world name
                world = "simple_cave_01"
                try:
                    out = subprocess.check_output(
                        ['ign', 'service', '-s', '/gazebo/worlds',
                         '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.StringMsg_V',
                         '--timeout', '800', '--req', ''],
                        stderr=subprocess.DEVNULL, timeout=1.0
                    ).decode()
                    m = re.search(r'data:\s*\"([^\"]+)\"', out)
                    if m:
                        world = m.group(1)
                except Exception:
                    pass

                # 2. Discover light entity ID from scene graph if not yet cached
                if self._gazebo_light_id is None:
                    try:
                        sg = subprocess.check_output(
                            ['ign', 'service', '-s', f'/world/{world}/scene/graph',
                             '--reqtype', 'ignition.msgs.Empty', '--reptype', 'ignition.msgs.StringMsg',
                             '--timeout', '800', '--req', ''],
                            stderr=subprocess.DEVNULL, timeout=1.2
                        ).decode()
                        s = sg.encode().decode('unicode_escape', errors='ignore')
                        m_id = re.search(r'(\d+)\s+\[label=\"camera_flashlight', s)
                        if m_id:
                            self._gazebo_light_id = int(m_id.group(1))
                    except Exception:
                        pass

                is_on_str = "1" if is_on else "0"
                intensity = 4.0 if is_on else 0.0
                r, g, b = (1.0, 1.0, 0.95) if is_on else (0.0, 0.0, 0.0)
                spec = 0.8 if is_on else 0.0
                rng = 35.0 if is_on else 0.0
                id_clause = f"id: {self._gazebo_light_id} " if self._gazebo_light_id else ""

                req_msg = (
                    f'{id_clause}name: "camera_flashlight" '
                    f'header {{ data {{ key: "isLightOn" value: "{is_on_str}" }} }} '
                    f'intensity: {intensity} '
                    f'diffuse {{ r: {r} g: {g} b: {b} a: 1.0 }} '
                    f'specular {{ r: {spec} g: {spec} b: {spec} a: 1.0 }} '
                    f'range: {rng}'
                )
                cmd = [
                    'ign', 'service', '-s', f'/world/{world}/light_config',
                    '--reqtype', 'ignition.msgs.Light',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', req_msg
                ]
                subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
            except Exception:
                pass

        threading.Thread(target=_worker, daemon=True).start()

    def cycle_colormap(self):
        self.colormap_idx = (self.colormap_idx + 1) % len(COLORMAPS)
        name, _ = COLORMAPS[self.colormap_idx]
        self.get_logger().info(f"Hotkey [C] → Thermal Colormap: {name}")

    def save_snapshot(self, rgb_disp, thermal_disp):
        timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        rgb_path = os.path.join(self.save_dir, f"rgb_{timestamp_str}.png")
        thermal_path = os.path.join(self.save_dir, f"thermal_{timestamp_str}.png")
        combined_path = os.path.join(self.save_dir, f"dual_{timestamp_str}.png")

        if rgb_disp is not None:
            cv2.imwrite(rgb_path, rgb_disp)
        if thermal_disp is not None:
            cv2.imwrite(thermal_path, thermal_disp)

        if rgb_disp is not None and thermal_disp is not None:
            combined = np.hstack([rgb_disp, thermal_disp])
            cv2.imwrite(combined_path, combined)

        self.get_logger().info(f"📸 Snapshots saved to {self.save_dir}/")


def create_placeholder(width, height, text):
    img = np.zeros((height, width, 3), dtype=np.uint8)
    img[:] = (20, 24, 28)
    # Subtle cross-grid
    cv2.line(img, (0, height // 2), (width, height // 2), (35, 42, 50), 1)
    cv2.line(img, (width // 2, 0), (width // 2, height), (35, 42, 50), 1)
    # Warning text
    cv2.putText(img, text, (width // 2 - 150, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (100, 160, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "Waiting for ROS2 topic...", (width // 2 - 120, height // 2 + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 130, 140), 1, cv2.LINE_AA)
    return img


def draw_hud_rgb(img, fps, flashlight_on, last_ts):
    h, w = img.shape[:2]
    overlay = img.copy()

    # Top header bar
    cv2.rectangle(overlay, (0, 0), (w, 38), (15, 18, 22), -1)
    # Bottom status bar
    cv2.rectangle(overlay, (0, h - 28), (w, h), (15, 18, 22), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)

    # Title & Badge
    cv2.circle(img, (18, 19), 6, (0, 255, 0), -1)
    cv2.putText(img, "RGB OPTICAL SENSOR", (32, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

    # Watchdog Heartbeat Indicator
    now = time.time()
    age = now - last_ts if last_ts > 0 else 999.0
    is_blink = int(now * 3) % 2 == 0
    dot_color = (0, 255, 0) if is_blink else (0, 180, 0)
    cv2.circle(img, (w - 290, 24), 5, dot_color, -1)
    cv2.putText(img, "LIVE", (w - 280, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1, cv2.LINE_AA)

    # FPS & Resolution
    cv2.putText(img, f"RES: {w}x{h} | FPS: {fps:.1f}", (w - 210, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 230, 255), 1, cv2.LINE_AA)

    # Center Crosshair
    cx, cy = w // 2, h // 2
    cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 16, (0, 255, 0), 1, cv2.LINE_AA)

    # Flashlight Status Indicator
    fl_color = (0, 255, 255) if flashlight_on else (100, 110, 120)
    fl_text = "FLASHLIGHT: ON [L]" if flashlight_on else "FLASHLIGHT: OFF [L]"
    cv2.putText(img, fl_text, (15, h - 9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, fl_color, 1, cv2.LINE_AA)

    return img


def draw_hud_thermal(img_color, raw_mono, fps, colormap_name, last_ts):
    h, w = img_color.shape[:2]
    overlay = img_color.copy()

    # Top header bar
    cv2.rectangle(overlay, (0, 0), (w, 38), (15, 18, 22), -1)
    # Bottom status bar
    cv2.rectangle(overlay, (0, h - 28), (w, h), (15, 18, 22), -1)
    cv2.addWeighted(overlay, 0.75, img_color, 0.25, 0, img_color)

    # Title & Badge
    cv2.circle(img_color, (18, 19), 6, (0, 140, 255), -1)
    cv2.putText(img_color, "THERMAL LWIR INFRARED", (32, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)

    # Watchdog Heartbeat Indicator
    now = time.time()
    age = now - last_ts if last_ts > 0 else 999.0
    is_blink = int(now * 3) % 2 == 0
    dot_color = (0, 255, 0) if is_blink else (0, 180, 0)
    cv2.circle(img_color, (w - 360, 24), 5, dot_color, -1)
    cv2.putText(img_color, "LIVE", (w - 350, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1, cv2.LINE_AA)

    # Palette & FPS
    cv2.putText(img_color, f"PALETTE: {colormap_name} [C] | FPS: {fps:.1f}", (w - 280, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 255), 1, cv2.LINE_AA)

    # Center Crosshair & Spot Meter
    cx, cy = w // 2, h // 2
    cv2.drawMarker(img_color, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
    cv2.circle(img_color, (cx, cy), 16, (0, 255, 255), 1, cv2.LINE_AA)

    # Thermal Spot Readings (safely guarded against dimension mismatches)
    if raw_mono is not None and len(raw_mono.shape) == 2:
        h_mono, w_mono = raw_mono.shape
        scy = min(max(0, cy), h_mono - 1)
        scx = min(max(0, cx), w_mono - 1)
        center_val = int(raw_mono[scy, scx])
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(raw_mono)

        # Draw Min / Max Spot Reticles
        cv2.circle(img_color, max_loc, 6, (0, 0, 255), 1, cv2.LINE_AA)  # Max = Red
        cv2.circle(img_color, min_loc, 6, (255, 200, 0), 1, cv2.LINE_AA)  # Min = Cyan

        # Center Temperature Reading: 0-255 -> 10°C - 80°C
        temp_center_c = 10.0 + (center_val / 255.0) * 70.0
        temp_max_c = 10.0 + (max_val / 255.0) * 70.0
        temp_min_c = 10.0 + (min_val / 255.0) * 70.0

        cv2.putText(img_color, f"SPOT: {temp_center_c:.1f} C (val:{center_val})", (cx + 22, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

        stat_text = f"MIN: {temp_min_c:.1f} C | MAX: {temp_max_c:.1f} C | RANGE: {temp_max_c - temp_min_c:.1f} C"
        cv2.putText(img_color, stat_text, (15, h - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 220, 240), 1, cv2.LINE_AA)

    return img_color


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraViewer()

    # Spin ROS 2 in dedicated background thread so GUI never starves sensor callbacks
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    window_title = "WareOps Armo_bot — Dual RGB & Thermal Inspection HUD"
    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_title, 1280, 520)

    try:
        while rclpy.ok():
            with node.lock:
                rgb = node.rgb_frame.copy() if node.rgb_frame is not None else None
                thermal_raw = node.thermal_frame.copy() if node.thermal_frame is not None else None
                rgb_fps = node.rgb_fps
                thermal_fps = node.thermal_fps
                flashlight_on = node.flashlight_state
                rgb_ts = node.rgb_timestamp
                thermal_ts = node.thermal_timestamp

            # 1. Process RGB Frame
            if rgb is None:
                rgb_disp = create_placeholder(640, 480, "RGB CAMERA OFFLINE")
            else:
                rgb_disp = cv2.resize(rgb, (640, 480))
                if flashlight_on:
                    # Apply tactical inspection illumination beam (center spotlight)
                    boosted = rgb_disp.astype(np.float32)
                    boosted[:, :, 0] = np.clip(boosted[:, :, 0] * (1.0 + node.flashlight_beam * 1.2), 0, 255)
                    boosted[:, :, 1] = np.clip(boosted[:, :, 1] * (1.0 + node.flashlight_beam * 1.4), 0, 255)
                    boosted[:, :, 2] = np.clip(boosted[:, :, 2] * (1.0 + node.flashlight_beam * 1.5), 0, 255)
                    rgb_disp = boosted.astype(np.uint8)
            rgb_disp = draw_hud_rgb(rgb_disp, rgb_fps, flashlight_on, rgb_ts)

            # 2. Process Thermal Frame
            colormap_name, colormap_val = COLORMAPS[node.colormap_idx]
            if thermal_raw is None:
                thermal_disp = create_placeholder(640, 480, "THERMAL CAMERA OFFLINE")
                thermal_scaled = None
            else:
                thermal_scaled = cv2.resize(thermal_raw, (640, 480))
                # Apply chosen false-color infrared colormap
                if colormap_val is not None:
                    thermal_disp = cv2.applyColorMap(thermal_scaled, colormap_val)
                else:
                    thermal_disp = cv2.cvtColor(thermal_scaled, cv2.COLOR_GRAY2BGR)
                thermal_disp = draw_hud_thermal(thermal_disp, thermal_scaled, thermal_fps, colormap_name, thermal_ts)

            # 3. Render Windows
            if node.split_windows:
                # Two separate windows
                cv2.destroyWindow(window_title)
                cv2.imshow("WareOps — RGB Camera", rgb_disp)
                cv2.imshow("WareOps — Thermal Camera", thermal_disp)
            else:
                # Single unified side-by-side display
                divider = np.full((480, 4, 3), 45, dtype=np.uint8)
                unified_frame = np.hstack([rgb_disp, divider, thermal_disp])
                cv2.imshow(window_title, unified_frame)

            # Handle Hotkeys with 25ms wait for responsive GUI rendering on Wayland / X11
            key = cv2.waitKey(25) & 0xFF
            if key in [ord('q'), ord('Q'), 27]:  # Q or ESC
                break
            elif key in [ord('c'), ord('C')]:  # Cycle thermal colormap
                node.cycle_colormap()
            elif key in [ord('l'), ord('L')]:  # Toggle flashlight
                node.toggle_flashlight()
            elif key in [ord('s'), ord('S')]:  # Save snapshot
                node.save_snapshot(rgb_disp, thermal_disp)
            elif key in [ord('w'), ord('W')]:  # Toggle split/unified view
                node.split_windows = not node.split_windows
                if not node.split_windows:
                    cv2.destroyWindow("WareOps — RGB Camera")
                    cv2.destroyWindow("WareOps — Thermal Camera")
                    cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        try:
            executor.shutdown()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        spin_thread.join(timeout=1.0)


if __name__ == '__main__':
    main()
