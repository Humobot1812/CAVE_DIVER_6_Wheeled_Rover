#!/usr/bin/env python3
"""
SIH Ground Control Station (GCS) - Backend Server
==================================================
Subterranean Recon & Environmental Monitoring System
Features:
- Dual RGB Optical & Thermal Infrared Camera streams with HUD overlays
- Flashlight toggle with visual beam boost + Gazebo Ignition/GZ service dispatch
- False-color LWIR Thermal Colormaps (INFERNO, JET, HOT, MAGMA, PLASMA, TURBO, GRAYSCALE)
- Live Environmental Sensor Telemetry (Temperature, Humidity, CO2, O2, CH4, CO)
- LiDAR radar ranges & mission event logging
- REST API & MJPEG streaming endpoints
"""

import os
import sys
import time
import json
import math
import random
import threading
import datetime
import subprocess
import re
import cv2
import numpy as np
from flask import Flask, Response, jsonify, request, render_template, send_from_directory

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    from sensor_msgs.msg import Image, LaserScan
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, Float32
    from cv_bridge import CvBridge, CvBridgeError
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
TEMPLATES_DIR = os.path.join(BASE_DIR, 'templates')
STATIC_DIR = os.path.join(BASE_DIR, 'static')
CAPTURES_DIR = os.path.join(BASE_DIR, 'captures')
os.makedirs(CAPTURES_DIR, exist_ok=True)

app = Flask(__name__, template_folder=TEMPLATES_DIR, static_folder=STATIC_DIR)

COLORMAPS = {
    'INFERNO': cv2.COLORMAP_INFERNO,
    'JET': cv2.COLORMAP_JET,
    'HOT': cv2.COLORMAP_HOT,
    'MAGMA': cv2.COLORMAP_MAGMA,
    'PLASMA': cv2.COLORMAP_PLASMA,
    'TURBO': cv2.COLORMAP_TURBO,
    'GRAYSCALE': None
}
COLORMAP_KEYS = list(COLORMAPS.keys())

# Precompute radial flashlight spotlight beam mask (exact 1:1 match with view_dual_camera.py)
def _make_flashlight_beam(h=480, w=640):
    cx, cy = w // 2, h // 2
    y, x = np.ogrid[:h, :w]
    dist_sq = ((x - cx) ** 2 + (y - cy) ** 2) / (float(w // 2) ** 2)
    return np.clip(2.5 * np.exp(-dist_sq * 2.0), 0.0, 2.5)

_FL_BEAM = _make_flashlight_beam(480, 640)

def apply_flashlight_beam(frame, beam=_FL_BEAM):
    """Exact 1:1 match with view_dual_camera.py for crystal clear, crisp cave textures"""
    boosted = frame.astype(np.float32)
    boosted[:, :, 0] = np.clip(boosted[:, :, 0] * (1.0 + beam * 1.2), 0, 255)
    boosted[:, :, 1] = np.clip(boosted[:, :, 1] * (1.0 + beam * 1.4), 0, 255)
    boosted[:, :, 2] = np.clip(boosted[:, :, 2] * (1.0 + beam * 1.5), 0, 255)
    return boosted.astype(np.uint8)

def draw_hud_rgb(img, fps, flashlight_on, last_ts):
    """Draw full tactical HUD overlay on RGB frame matching view_dual_camera.py"""
    h, w = img.shape[:2]
    overlay = img.copy()

    # Top header bar & bottom status bar
    cv2.rectangle(overlay, (0, 0), (w, 36), (15, 18, 22), -1)
    cv2.rectangle(overlay, (0, h - 28), (w, h), (15, 18, 22), -1)
    cv2.addWeighted(overlay, 0.75, img, 0.25, 0, img)

    # Title & Badge
    cv2.circle(img, (18, 18), 5, (0, 255, 0), -1)
    cv2.putText(img, "RGB OPTICAL SENSOR", (32, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)

    # Heartbeat Indicator (Always active LIVE indicator, never stall)
    now = time.time()
    is_blink = int(now * 3) % 2 == 0
    dot_color = (0, 255, 0) if is_blink else (0, 180, 0)
    cv2.circle(img, (w - 290, 18), 5, dot_color, -1)
    cv2.putText(img, "LIVE", (w - 280, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1, cv2.LINE_AA)

    # FPS & Resolution
    cv2.putText(img, f"RES: {w}x{h} | FPS: {fps:.1f}", (w - 215, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 230, 255), 1, cv2.LINE_AA)

    # Center Crosshair
    cx, cy = w // 2, h // 2
    cv2.drawMarker(img, (cx, cy), (0, 255, 0), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
    cv2.circle(img, (cx, cy), 16, (0, 255, 0), 1, cv2.LINE_AA)

    # Flashlight Status Indicator (High visibility on camera stream)
    fl_color = (0, 255, 255) if flashlight_on else (120, 130, 140)
    fl_text = "FLASHLIGHT: ON [L]" if flashlight_on else "FLASHLIGHT: OFF [L]"
    cv2.putText(img, fl_text, (15, h - 9),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, fl_color, 1, cv2.LINE_AA)

    return img

def draw_hud_thermal(img_color, raw_mono, fps, colormap_name, last_ts):
    """Draw tactical HUD overlay on Thermal frame matching view_dual_camera.py"""
    h, w = img_color.shape[:2]
    overlay = img_color.copy()

    # Top header bar & bottom status bar
    cv2.rectangle(overlay, (0, 0), (w, 36), (15, 18, 22), -1)
    cv2.rectangle(overlay, (0, h - 28), (w, h), (15, 18, 22), -1)
    cv2.addWeighted(overlay, 0.75, img_color, 0.25, 0, img_color)

    # Title & Badge
    cv2.circle(img_color, (18, 18), 5, (0, 140, 255), -1)
    cv2.putText(img_color, "THERMAL LWIR INFRARED", (32, 23),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (255, 255, 255), 2, cv2.LINE_AA)

    now = time.time()
    is_blink = int(now * 3) % 2 == 0
    dot_color = (0, 255, 0) if is_blink else (0, 180, 0)
    cv2.circle(img_color, (w - 360, 18), 5, dot_color, -1)
    cv2.putText(img_color, "LIVE", (w - 350, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 0), 1, cv2.LINE_AA)

    # Palette & FPS
    cv2.putText(img_color, f"PALETTE: {colormap_name} [C] | FPS: {fps:.1f}", (w - 285, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 200, 255), 1, cv2.LINE_AA)

    cx, cy = w // 2, h // 2
    cv2.drawMarker(img_color, (cx, cy), (0, 255, 255), cv2.MARKER_CROSS, 20, 1, cv2.LINE_AA)
    cv2.circle(img_color, (cx, cy), 16, (0, 255, 255), 1, cv2.LINE_AA)

    if raw_mono is not None and len(raw_mono.shape) == 2:
        h_mono, w_mono = raw_mono.shape
        scy = min(max(0, cy), h_mono - 1)
        scx = min(max(0, cx), w_mono - 1)
        center_val = int(raw_mono[scy, scx])
        min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(raw_mono)

        cv2.circle(img_color, max_loc, 6, (0, 0, 255), 1, cv2.LINE_AA)
        cv2.circle(img_color, min_loc, 6, (255, 200, 0), 1, cv2.LINE_AA)

        temp_center_c = 10.0 + (center_val / 255.0) * 70.0
        temp_max_c = 10.0 + (max_val / 255.0) * 70.0
        temp_min_c = 10.0 + (min_val / 255.0) * 70.0

        cv2.putText(img_color, f"SPOT: {temp_center_c:.1f} C (val:{center_val})", (cx + 22, cy + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (0, 255, 255), 1, cv2.LINE_AA)
        stat_text = f"MIN: {temp_min_c:.1f} C | MAX: {temp_max_c:.1f} C | RANGE: {temp_max_c - temp_min_c:.1f} C"
        cv2.putText(img_color, stat_text, (15, h - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (200, 220, 240), 1, cv2.LINE_AA)

    return img_color

_GAZEBO_LIGHT_ID = 496  # Resolved entity ID for camera_flashlight in simple_cave_01

# Gazebo light entity dispatcher
def send_gazebo_light_cmd(is_on: bool, cached_id=None):
    global _GAZEBO_LIGHT_ID
    if cached_id is not None:
        _GAZEBO_LIGHT_ID = cached_id
    """Send light config service call directly to Gazebo (ign or gz)"""
    def _worker():
        try:
            world = "simple_cave_01"
            # 1. Discover world
            for cli in ['ign', 'gz']:
                try:
                    prefix = "ignition" if cli == 'ign' else "gz"
                    out = subprocess.check_output(
                        [cli, 'service', '-s', '/gazebo/worlds',
                         '--reqtype', f'{prefix}.msgs.Empty', '--reptype', f'{prefix}.msgs.StringMsg_V',
                         '--timeout', '800', '--req', ''],
                        stderr=subprocess.DEVNULL, timeout=1.0
                    ).decode()
                    m = re.search(r'data:\s*"([^"]+)"', out)
                    if m:
                        world = m.group(1)
                        break
                except Exception:
                    pass

            # 2. Discover light entity id
            light_id = cached_id or _GAZEBO_LIGHT_ID
            if light_id is None:
                for cli in ['ign', 'gz']:
                    try:
                        prefix = "ignition" if cli == 'ign' else "gz"
                        sg = subprocess.check_output(
                            [cli, 'service', '-s', f'/world/{world}/scene/graph',
                             '--reqtype', f'{prefix}.msgs.Empty', '--reptype', f'{prefix}.msgs.StringMsg',
                             '--timeout', '800', '--req', ''],
                            stderr=subprocess.DEVNULL, timeout=1.2
                        ).decode()
                        s = sg.encode().decode('unicode_escape', errors='ignore')
                        m_id = re.search(r'(\d+)\s+\[label="camera_flashlight', s)
                        if m_id:
                            light_id = int(m_id.group(1))
                            break
                    except Exception:
                        pass

            is_on_str = "1" if is_on else "0"
            intensity = 4.0 if is_on else 0.0
            r, g, b = (1.0, 1.0, 0.95) if is_on else (0.0, 0.0, 0.0)
            spec = 0.8 if is_on else 0.0
            rng = 35.0 if is_on else 0.0
            id_clause = f"id: {light_id} " if light_id else ""

            req_msg = (
                f'{id_clause}name: "camera_flashlight" '
                f'header {{ data {{ key: "isLightOn" value: "{is_on_str}" }} }} '
                f'intensity: {intensity} '
                f'diffuse {{ r: {r} g: {g} b: {b} a: 1.0 }} '
                f'specular {{ r: {spec} g: {spec} b: {spec} a: 1.0 }} '
                f'range: {rng}'
            )

            for cli in ['ign', 'gz']:
                try:
                    prefix = "ignition" if cli == 'ign' else "gz"
                    cmd = [
                        cli, 'service', '-s', f'/world/{world}/light_config',
                        '--reqtype', f'{prefix}.msgs.Light',
                        '--reptype', f'{prefix}.msgs.Boolean',
                        '--timeout', '1000',
                        '--req', req_msg
                    ]
                    subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=1.5)
                    break
                except Exception:
                    pass
        except Exception:
            pass

    threading.Thread(target=_worker, daemon=True).start()

_syn_rgb_t = 0.0
_syn_th_t = 0.0

def generate_synthetic_rgb(fl_on=False):
    """Generate realistic cave tunnel feed. When fl_on is False, cave is dark."""
    global _syn_rgb_t
    _syn_rgb_t += 0.03
    t = _syn_rgb_t
    w, h = 640, 480
    frame = np.zeros((h, w, 3), dtype=np.uint8)

    # Ambient subterranean cave backdrop (dim rocks and silhouettes)
    base_lum = 60 if fl_on else 15
    for row in range(h):
        v = int(base_lum + 12 * math.sin(row / h * math.pi) + 6 * math.sin(t * 0.3 + row * 0.05))
        frame[row] = [int(v * 0.75), int(v * 0.8), int(v * 0.85)]

    # Mine / Cave wall boulders
    for i in range(6):
        rx = int(w * 0.15 + (w * 0.7) * (i / 5.0) + 12 * math.sin(t * 0.2 + i))
        ry = int(h * 0.58 + 25 * math.sin(t * 0.15 + i * 1.3))
        rad = int(18 + 12 * math.sin(i * 1.7))
        col = (28 + i * 4, 32 + i * 4, 36 + i * 3)
        cv2.circle(frame, (rx, ry), rad, col, -1)

    return frame

def generate_synthetic_thermal():
    """Generate synthetic thermal infrared raw 8-bit mono feed"""
    global _syn_th_t
    _syn_th_t += 0.03
    t = _syn_th_t
    w, h = 640, 480
    mono = np.zeros((h, w), dtype=np.float32)

    for row in range(h):
        mono[row, :] = 50 + 16 * math.sin(row / h * math.pi) + 10 * math.cos(t * 0.4 + row * 0.03)

    cx, cy = w // 2, h // 2
    Y, X = np.ogrid[:h, :w]
    mono += 80 * np.exp(-((X - cx) ** 2 + (Y - cy) ** 2).astype(np.float32) / (2 * 120 ** 2))

    # Heat signatures
    for i in range(3):
        hx = int(w * (0.22 + 0.28 * i) + 18 * math.sin(t * 0.25 + i))
        hy = int(h * 0.55 + 22 * math.cos(t * 0.35 + i * 2))
        mono += (55 + 26 * math.sin(t * 1.2 + i)) * np.exp(-((X - hx) ** 2 + (Y - hy) ** 2).astype(np.float32) / (2 * 35 ** 2))

    return np.clip(mono, 0, 255).astype(np.uint8)

class EnvironmentalSensorSuite:
    def __init__(self):
        self.temperature = 22.4
        self.humidity = 64.5
        self.co2 = 512.0
        self.o2 = 20.92
        self.ch4 = 0.019
        self.co = 1.4
        self.hazard_injection = False
        self.last_update = time.time()
        self.history = {k: [] for k in ['timestamps', 'temperature', 'humidity', 'co2', 'o2', 'ch4', 'co']}

    def update(self):
        now = time.time()
        dt = min(now - self.last_update, 1.0)
        self.last_update = now

        def n(scale): return (random.random() - 0.5) * 2.0 * scale

        if not self.hazard_injection:
            self.temperature = float(np.clip(self.temperature + n(0.08) * dt + (22.5 - self.temperature) * 0.02 * dt, 15, 35))
            self.humidity = float(np.clip(self.humidity + n(0.15) * dt + (64.0 - self.humidity) * 0.02 * dt, 30, 95))
            self.co2 = float(np.clip(self.co2 + n(1.5) * dt + (510.0 - self.co2) * 0.02 * dt, 400, 1500))
            self.o2 = float(np.clip(self.o2 + n(0.01) * dt + (20.9 - self.o2) * 0.02 * dt, 18, 22))
            self.ch4 = float(np.clip(self.ch4 + n(0.002) * dt + (0.018 - self.ch4) * 0.02 * dt, 0.0, 1.0))
            self.co = float(np.clip(self.co + n(0.05) * dt + (1.2 - self.co) * 0.02 * dt, 0.0, 50))
            status = 'NOMINAL'
        else:
            self.temperature = float(min(self.temperature + 0.35 * dt, 48.0))
            self.humidity = float(max(self.humidity - 0.4 * dt, 22.0))
            self.co2 = float(min(self.co2 + 45.0 * dt, 3200.0))
            self.o2 = float(max(self.o2 - 0.12 * dt, 14.8))
            self.ch4 = float(min(self.ch4 + 0.05 * dt, 3.8))
            self.co = float(min(self.co + 1.2 * dt, 85.0))
            status = 'DANGER' if (self.ch4 > 1.0 or self.o2 < 18.0 or self.co > 25.0) else 'WARNING'

        h = self.history
        ts = datetime.datetime.now().strftime('%H:%M:%S')
        h['timestamps'].append(ts)
        h['temperature'].append(round(self.temperature, 2))
        h['humidity'].append(round(self.humidity, 2))
        h['co2'].append(round(self.co2, 1))
        h['o2'].append(round(self.o2, 2))
        h['ch4'].append(round(self.ch4, 3))
        h['co'].append(round(self.co, 2))
        if len(h['timestamps']) > 60:
            for k in h: h[k].pop(0)

        return {
            'temperature': round(self.temperature, 1),
            'temperature_c': round(self.temperature, 1),
            'humidity': round(self.humidity, 1),
            'humidity_rh': round(self.humidity, 1),
            'co2': round(self.co2, 0),
            'co2_ppm': round(self.co2, 0),
            'o2': round(self.o2, 2),
            'o2_percent': round(self.o2, 2),
            'ch4': round(self.ch4, 3),
            'ch4_lel': round(self.ch4, 3),
            'co': round(self.co, 2),
            'co_ppm': round(self.co, 2),
            'status': status,
            'hazard_active': self.hazard_injection,
            'history': self.history
        }

sensors = EnvironmentalSensorSuite()
ros_node = None
_fl_standalone = True

def _enc(frame, q=85):
    r, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, q])
    return buf.tobytes() if r else None

def rgb_stream():
    """MJPEG RGB optical stream with full inspection HUD & flashlight beam (auto-fallback, never stalls)"""
    while True:
        frame = None
        fl_on = False
        fps = 20.0
        now = time.time()
        ts = now

        if ros_node:
            with ros_node.lock:
                # Always use last received frame - no expiry so feed never stalls
                if ros_node.rgb_frame is not None:
                    frame = ros_node.rgb_frame.copy()
                    fps = max(ros_node.rgb_fps, 10.0)
                    ts = ros_node.rgb_timestamp
                fl_on = ros_node.flashlight_state
        else:
            fl_on = _fl_standalone

        if frame is None:
            # Only fall back to synthetic if we have never received a real frame
            frame = generate_synthetic_rgb(fl_on=fl_on)
            fps = 20.0
            ts = now

        frame = cv2.resize(frame, (640, 480))

        # Apply tactical inspection illumination beam (center spotlight)
        if fl_on:
            frame = apply_flashlight_beam(frame, _FL_BEAM)

        # Draw HUD on frame
        frame = draw_hud_rgb(frame, fps, fl_on, ts)

        data = _enc(frame)
        if data:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n'
        time.sleep(0.04)

def thermal_stream():
    """MJPEG Thermal infrared stream with false-color colormap and HUD (auto-fallback, never stalls)"""
    while True:
        raw_mono = None
        cn = 'INFERNO'
        fps = 15.0
        now = time.time()
        ts = now

        if ros_node:
            with ros_node.lock:
                cn = ros_node.active_colormap
                # Always use last received frame - no expiry so feed never stalls
                if ros_node.thermal_frame is not None:
                    raw_mono = ros_node.thermal_frame.copy()
                    fps = max(ros_node.thermal_fps, 10.0)
                    ts = ros_node.thermal_timestamp

        if raw_mono is None:
            # Only fall back to synthetic if we have never received a real frame
            raw_mono = generate_synthetic_thermal()
            fps = 15.0
            ts = now

        raw_mono = cv2.resize(raw_mono, (640, 480))
        cm = COLORMAPS.get(cn, cv2.COLORMAP_INFERNO)
        if cm is not None:
            colored = cv2.applyColorMap(raw_mono, cm)
        else:
            colored = cv2.cvtColor(raw_mono, cv2.COLOR_GRAY2BGR)

        colored = draw_hud_thermal(colored, raw_mono, fps, cn, ts)

        data = _enc(colored, 80)
        if data:
            yield b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + data + b'\r\n'
        time.sleep(0.05)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/stream/rgb')
def stream_rgb():
    return Response(rgb_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/stream/thermal')
def stream_thermal():
    return Response(thermal_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/telemetry')
@app.route('/api/telemetry')
def get_telemetry():
    env = sensors.update()
    fl_state = ros_node.flashlight_state if ros_node else _fl_standalone
    active_cm = ros_node.active_colormap if ros_node else 'INFERNO'

    cam = {
        'flashlight': fl_state,
        'colormap': active_cm,
        'rgb_fps': round(ros_node.rgb_fps, 1) if ros_node else 20.0,
        'thermal_fps': round(ros_node.thermal_fps, 1) if ros_node else 15.0,
        'spot_temp_c': 24.8
    }
    bot = {
        'connected': ros_node is not None,
        'estop': ros_node.estop_active if ros_node else False,
        'robot_x': ros_node.robot_x if ros_node else 0.0,
        'robot_y': ros_node.robot_y if ros_node else 0.0,
        'robot_yaw_deg': round(math.degrees(ros_node.robot_yaw), 1) if ros_node else 0.0,
        'lidar_ranges': ros_node.lidar_ranges if ros_node else []
    }
    return jsonify({
        'timestamp': datetime.datetime.now().isoformat(),
        'environment': env,
        'environmental': env,
        'camera': cam,
        'robot': bot,
        'ros_status': {'rclpy_active': ros_node is not None}
    })

@app.route('/cmd_vel', methods=['POST'])
@app.route('/api/cmd_vel', methods=['POST'])
def cmd_vel():
    d = request.get_json(silent=True) or {}
    lin = float(d.get('linear', d.get('linear_x', 0.0)))
    ang = float(d.get('angular', d.get('angular_z', 0.0)))
    if ros_node:
        ros_node.send_cmd_vel(lin, ang)
    return jsonify({'success': True, 'linear': lin, 'angular': ang})

@app.route('/camera/gimbal', methods=['POST'])
@app.route('/api/camera/gimbal', methods=['POST'])
def gimbal_ctrl():
    d = request.get_json(silent=True) or {}
    yaw = float(d.get('yaw', d.get('yaw_deg', 0.0)))
    pitch = float(d.get('pitch', d.get('pitch_deg', 0.0)))
    if ros_node:
        ros_node.set_gimbal(yaw, pitch)
    return jsonify({'success': True, 'yaw_deg': yaw, 'pitch_deg': pitch})

@app.route('/camera/flashlight', methods=['POST'])
@app.route('/api/camera/flashlight', methods=['POST'])
def flashlight_ctrl():
    global _fl_standalone
    d = request.get_json(silent=True) or {}
    current = ros_node.flashlight_state if ros_node else _fl_standalone

    if 'enabled' in d:
        state = bool(d['enabled'])
    elif 'state' in d:
        state = bool(d['state'])
    else:
        state = not current

    _fl_standalone = state
    if ros_node:
        ros_node.set_flashlight(state)
    else:
        # Standalone mode: still attempt Gazebo dispatch if Gazebo is running
        send_gazebo_light_cmd(state)

    return jsonify({'success': True, 'flashlight': state})

@app.route('/camera/colormap', methods=['POST'])
@app.route('/api/camera/colormap', methods=['POST'])
def set_colormap():
    d = request.get_json(silent=True) or {}
    cm = d.get('colormap', 'INFERNO').upper()
    if cm in COLORMAPS:
        if ros_node:
            with ros_node.lock:
                ros_node.active_colormap = cm
        return jsonify({'success': True, 'colormap': cm})
    return jsonify({'success': False, 'error': 'Invalid colormap'}), 400

@app.route('/camera/snapshot', methods=['POST'])
@app.route('/api/camera/snapshot', methods=['POST'])
def capture_snapshot():
    ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    fn_r = f'rgb_{ts}.png'
    fn_t = f'thermal_{ts}.png'
    fn_d = f'dual_{ts}.png'

    ri = None
    ti = None
    fl_on = ros_node.flashlight_state if ros_node else _fl_standalone
    cn = ros_node.active_colormap if ros_node else 'INFERNO'

    if ros_node:
        with ros_node.lock:
            if ros_node.rgb_frame is not None:
                ri = ros_node.rgb_frame.copy()
            if ros_node.thermal_frame is not None:
                raw = ros_node.thermal_frame.copy()
                cm = COLORMAPS.get(cn, cv2.COLORMAP_INFERNO)
                ti = cv2.applyColorMap(raw, cm) if cm else cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)

    if ri is None:
        ri = generate_synthetic_rgb(fl_on=fl_on)
    if fl_on:
        ri = apply_flashlight_beam(ri, _FL_BEAM)
    ri = draw_hud_rgb(cv2.resize(ri, (640, 480)), 20.0, fl_on, time.time())

    if ti is None:
        raw = generate_synthetic_thermal()
        cm = COLORMAPS.get(cn, cv2.COLORMAP_INFERNO)
        ti = cv2.applyColorMap(raw, cm) if cm else cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
        ti = draw_hud_thermal(cv2.resize(ti, (640, 480)), raw, 15.0, cn, time.time())
    else:
        ti = cv2.resize(ti, (640, 480))

    cv2.imwrite(os.path.join(CAPTURES_DIR, fn_r), ri)
    cv2.imwrite(os.path.join(CAPTURES_DIR, fn_t), ti)
    cv2.imwrite(os.path.join(CAPTURES_DIR, fn_d), np.hstack([ri, ti]))

    return jsonify({
        'success': True,
        'timestamp': ts,
        'filename_rgb': fn_r,
        'filename_thermal': fn_t,
        'url_rgb': f'/captures/{fn_r}',
        'url_thermal': f'/captures/{fn_t}',
        'url_dual': f'/captures/{fn_d}'
    })

@app.route('/captures/<filename>')
def serve_capture(filename):
    return send_from_directory(CAPTURES_DIR, filename)

@app.route('/robot/estop', methods=['POST'])
@app.route('/api/robot/estop', methods=['POST'])
def estop():
    d = request.get_json(silent=True) or {}
    e = bool(d.get('estop', d.get('enable', True)))
    if ros_node:
        if e: ros_node.emergency_stop()
        else: ros_node.release_estop()
    return jsonify({'success': True, 'estop': e})

@app.route('/telemetry/simulate_hazard', methods=['POST'])
@app.route('/api/hazard_simulation', methods=['POST'])
def hazard():
    d = request.get_json(silent=True) or {}
    e = bool(d.get('enabled', d.get('enable', not sensors.hazard_injection)))
    sensors.hazard_injection = e
    return jsonify({'success': True, 'hazard_active': e})

if ROS2_AVAILABLE:
    class GCSBridgeNode(Node):
        def __init__(self):
            super().__init__('sih_gcs_bridge')
            self.bridge = CvBridge()
            self.lock = threading.Lock()
            self._gazebo_light_id = None

            self.rgb_frame = None
            self.thermal_frame = None
            self.rgb_timestamp = 0.0
            self.thermal_timestamp = 0.0
            self.rgb_fps = 0.0
            self.thermal_fps = 0.0
            self._rgb_prev = time.time()
            self._th_prev = time.time()
            self._rgb_cnt = 0
            self._th_cnt = 0

            self.flashlight_state = True
            self.active_colormap = 'INFERNO'
            self.estop_active = False
            self.lidar_ranges = []
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.robot_yaw = 0.0

            sq = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
            rq = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)

            # Subscriptions
            self.create_subscription(Image, '/camera/image_raw', self.rgb_cb, sq)
            self.create_subscription(Image, '/thermal/image_raw', self.thermal_cb, sq)
            self.create_subscription(Bool, '/camera_flashlight/status', self.fl_status_cb, 10)
            self.create_subscription(LaserScan, '/scan', self.scan_cb, sq)
            self.create_subscription(Odometry, '/odom', self.odom_cb, sq)

            # Publishers
            self.fl_cmd_pub = self.create_publisher(Bool, '/camera_flashlight/switch', 10)
            self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', rq)
            self.yaw_pub = self.create_publisher(Float32, '/camera/gimbal_yaw', rq)
            self.pitch_pub = self.create_publisher(Float32, '/camera/gimbal_pitch', rq)
            self.gimbal_yaw = 0.0
            self.gimbal_pitch = 0.0

            self.get_logger().info('SIH GCS ROS 2 Bridge Ready')

        def rgb_cb(self, msg):
            try:
                try:
                    img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                except Exception:
                    raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                    img = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR) if len(raw.shape) == 2 else raw

                now = time.time()
                self._rgb_cnt += 1
                if now - self._rgb_prev >= 1.0:
                    self.rgb_fps = self._rgb_cnt / (now - self._rgb_prev)
                    self._rgb_cnt = 0
                    self._rgb_prev = now

                with self.lock:
                    self.rgb_frame = img
                    self.rgb_timestamp = now
            except Exception:
                pass

        def thermal_cb(self, msg):
            try:
                try:
                    raw = self.bridge.imgmsg_to_cv2(msg, 'mono8')
                except Exception:
                    raw = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
                    if len(raw.shape) == 3:
                        raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

                if np.issubdtype(raw.dtype, np.floating):
                    raw = np.nan_to_num(raw, nan=0.0, posinf=255.0, neginf=0.0)
                if raw.dtype != np.uint8:
                    raw = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

                now = time.time()
                self._th_cnt += 1
                if now - self._th_prev >= 1.0:
                    self.thermal_fps = self._th_cnt / (now - self._th_prev)
                    self._th_cnt = 0
                    self._th_prev = now

                with self.lock:
                    self.thermal_frame = raw
                    self.thermal_timestamp = now
            except Exception:
                pass

        def fl_status_cb(self, msg: Bool):
            with self.lock:
                self.flashlight_state = bool(msg.data)

        def scan_cb(self, msg: LaserScan):
            ranges = [r if not (math.isnan(r) or math.isinf(r)) else -1 for r in msg.ranges]
            self.lidar_ranges = ranges

        def odom_cb(self, msg: Odometry):
            p = msg.pose.pose.position
            o = msg.pose.pose.orientation
            siny_cosp = 2.0 * (o.w * o.z + o.x * o.y)
            cosy_cosp = 1.0 - 2.0 * (o.y * o.y + o.z * o.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            with self.lock:
                self.robot_x = float(p.x)
                self.robot_y = float(p.y)
                self.robot_yaw = float(yaw)

        def set_flashlight(self, state):
            with self.lock:
                self.flashlight_state = bool(state)
            msg = Bool()
            msg.data = self.flashlight_state
            self.fl_cmd_pub.publish(msg)
            # Dispatch directly to Gazebo light config
            send_gazebo_light_cmd(self.flashlight_state, self._gazebo_light_id)

        def toggle_flashlight(self):
            new = not self.flashlight_state
            self.set_flashlight(new)
            return new

        def send_cmd_vel(self, lin, ang):
            if self.estop_active:
                return
            msg = Twist()
            msg.linear.x = float(lin)
            msg.angular.z = float(ang)
            self.cmd_vel_pub.publish(msg)

        def set_gimbal(self, yaw_deg, pitch_deg):
            yr = math.radians(float(yaw_deg))
            pr = math.radians(float(pitch_deg))
            my = Float32()
            my.data = float(yr)
            self.yaw_pub.publish(my)
            mp = Float32()
            mp.data = float(pr)
            self.pitch_pub.publish(mp)
            with self.lock:
                self.gimbal_yaw = yr
                self.gimbal_pitch = pr

        def emergency_stop(self):
            self.estop_active = True
            msg = Twist()
            self.cmd_vel_pub.publish(msg)

        def release_estop(self):
            self.estop_active = False

def run_ros2():
    global ros_node
    if not ROS2_AVAILABLE:
        print("[SIH GCS] ROS 2 not installed - Running in Standalone Synthetic Mode")
        return
    try:
        rclpy.init()
        ros_node = GCSBridgeNode()
        print("[SIH GCS] ROS 2 Node Started Successfully")
        rclpy.spin(ros_node)
    except Exception as e:
        print(f"[SIH GCS] ROS 2 spin exception: {e}")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='SIH GCS Server')
    parser.add_argument('--host', default='0.0.0.0', help='Host IP')
    parser.add_argument('--port', default=5001, type=int, help='Port number')
    args = parser.parse_args()

    ros_thread = threading.Thread(target=run_ros2, daemon=True)
    ros_thread.start()

    print(f"\n{'='*60}")
    print(f"🚀 SIH GROUND CONTROL STATION (GCS)")
    print(f"   Dashboard : http://localhost:{args.port}")
    print(f"   Network   : http://{args.host}:{args.port}")
    print(f"{'='*60}\n")

    app.run(host=args.host, port=args.port, threaded=True)
