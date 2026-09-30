#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Float64, Bool
import json
import subprocess
import threading
import re


class PS2Teleop(Node):

    def __init__(self):
        super().__init__('ps2_teleop')

        # Declare parameter to switch between real robot and simulation
        self.declare_parameter('use_sim', True)
        self.use_sim = self.get_parameter('use_sim').get_parameter_value().bool_value

        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10)

        # Choose cmd_vel topic based on mode
        if self.use_sim:
            cmd_vel_topic = '/cmd_vel'
        else:
            cmd_vel_topic = '/diff_cont/cmd_vel_unstamped'

        self.publisher = self.create_publisher(
            Twist,
            cmd_vel_topic,
            10)

        # Publisher: camera servo commands → esp32_wifi_bridge (real robot)
        self.servo_cmd_pub = self.create_publisher(
            String,
            '/camera_servo_cmd',
            10)

        # Gazebo joint position publishers (simulation only)
        if self.use_sim:
            self.gz_yaw_pub = self.create_publisher(
                Float64,
                '/camera_base_cylinder/cmd_pos',
                10)
            self.gz_pitch_pub = self.create_publisher(
                Float64,
                '/camera_cylinder_sphere/cmd_pos',
                10)

        # -----------------------------
        # Axis Mapping (Your Controller)
        # -----------------------------
        self.linear_axis = 1          # Left stick Up/Down
        self.angular_axis = 2         # Right stick Left/Right

        self.dpad_horizontal = 6      # Left/Right
        self.dpad_vertical = 7        # Up/Down

        # -----------------------------
        # Right-Side Face Button Mapping
        # PS2/Xbox style: A=0, B=1, X=3, Y=4
        # (Adjust indices here if your controller differs)
        # -----------------------------
        self.btn_A = 0   # Pitch DOWN  (+5 degrees, camera tilts down)
        self.btn_B = 1   # Yaw RIGHT   (-5 degrees towards right -90°)
        self.btn_X = 3  # Yaw LEFT    (+5 degrees towards left +90°)
        self.btn_Y = 4   # Pitch UP    (-5 degrees, camera tilts up)

        # -----------------------------
        # Select / Start Button Mapping
        # PS2 style: Select=8, Start=9
        # (Adjust indices here if your controller differs)
        # -----------------------------
        self.btn_SELECT = 10   # Go to PARKED position  (yaw=0°,   pitch=0°)
        self.btn_START  = 11   # Go to DEFAULT SCAN pos (yaw=-90°, pitch=0°)

        # -----------------------------
        # Status Shortcut Button Mapping
        # Buttons 6-9 publish preset /bot_status messages
        # -----------------------------




        self.btn_STATUS_6 = 60   # Reached Aisle_2/Row_1/Rack_1
        self.btn_STATUS_7 = 70   # Reached Aisle_1/Row_1/Rack_1
        self.btn_STATUS_8 = 80   # Reached Aisle_2/Row_1/Rack_2
        self.btn_STATUS_9 = 90   # Reached Aisle_1/Row_1/Rack_2

        self.STATUS_MESSAGES = {
            6: "Reached Aisle_2/Row_1/Rack_1",
            7: "Reached Aisle_1/Row_1/Rack_1",
            8: "Reached Aisle_2/Row_1/Rack_2",
            9: "Reached Aisle_1/Row_1/Rack_2",
        }

        # -----------------------------
        # Flashlight Button Mapping (Toggle Spotlight)
        # Button 5 = RB / R1 shoulder bumper
        # -----------------------------
        self.btn_FLASHLIGHT = 5
        self.prev_btn_FLASHLIGHT = 0
        self.flashlight_on = True  # Default to ON in dark simulation/cave
        self._gazebo_light_id = None

        # Flashlight ROS 2 Status publisher & Switch subscriber
        self.flashlight_status_pub = self.create_publisher(
            Bool,
            '/camera_flashlight/status',
            10)
        self.flashlight_sub = self.create_subscription(
            Bool,
            '/camera_flashlight/switch',
            self.flashlight_callback,
            10)

        # Preset positions — must match rack_scanner_node constants
        self.PARKED_YAW    = 0
        self.PARKED_PITCH  = 0
        self.DEFAULT_YAW   = -90
        self.DEFAULT_PITCH = 0

        # -----------------------------
        # Servo State
        # -----------------------------
        self.servo_yaw   = self.PARKED_YAW      # Start at parked yaw position (0°)
        self.servo_pitch = self.PARKED_PITCH    # Start at parked pitch position (0°)

        if self.use_sim:
            self.PITCH_MIN = -90         # Gazebo joint limit (tilt up)
            self.PITCH_MAX = 90          # Gazebo joint limit (tilt down)
            self.YAW_MIN   = -120        # Gazebo joint limit (right)
            self.YAW_MAX   = 90          # Gazebo joint limit (left)
        else:
            self.PITCH_MIN = 20          # Servo hardware limit (tilt up)
            self.PITCH_MAX = 160         # Servo hardware limit (tilt down)
            self.YAW_MIN   = 0
            self.YAW_MAX   = 180
        self.SERVO_STEP = 5         # Degrees per button press
        self.SERVO_SPEED = 20       # Speed value forwarded to ESP32

        # Debounce: track previous button states to detect press edges
        self.prev_btn_A      = 0
        self.prev_btn_B      = 0
        self.prev_btn_X      = 0
        self.prev_btn_Y      = 0
        self.prev_btn_SELECT = 0
        self.prev_btn_START  = 0
        self.prev_btn_6      = 0
        self.prev_btn_7      = 0
        self.prev_btn_8      = 0
        self.prev_btn_9      = 0

        # -----------------------------
        # Speed Limits
        # -----------------------------
        self.linear_scale = 0.5       # m/s
        self.angular_scale = 1.5      # rad/s

        self.min_linear = 0.1
        self.max_linear = 5.0

        self.min_angular = 0.2
        self.max_angular = 5.0

        # Debounce variables
        self.prev_dpad_vertical = 0
        self.prev_dpad_horizontal = 0

        mode_str = "SIMULATION" if self.use_sim else "REAL ROBOT"
        self.get_logger().info(f"PS2 Teleop Started — Mode: {mode_str}")
        self.get_logger().info(f"  cmd_vel topic: {cmd_vel_topic}")
        self.get_logger().info(
            f"Camera servo parked at yaw={self.servo_yaw}°, pitch={self.servo_pitch}° "
            f"| Step={self.SERVO_STEP}° | Pitch limits=[{self.PITCH_MIN}°, {self.PITCH_MAX}°]"
        )

        self.print_speed()

    def print_speed(self):
        self.get_logger().info(
            "\n"
            "=============================\n"
            f" Linear Speed  : {self.linear_scale:.2f} m/s\n"
            f" Angular Speed : {self.angular_scale:.2f} rad/s\n"
            "============================="
        )

    def joy_callback(self, msg):

        # -----------------------------
        # D-Pad Up / Down
        # -----------------------------
        dpad_v = int(msg.axes[self.dpad_vertical])

        if dpad_v != self.prev_dpad_vertical:

            if dpad_v == 1:

                self.linear_scale *= 1.10
                self.linear_scale = min(self.linear_scale,
                                        self.max_linear)

                self.print_speed()

            elif dpad_v == -1:

                self.linear_scale *= 0.90
                self.linear_scale = max(self.linear_scale,
                                        self.min_linear)

                self.print_speed()

        self.prev_dpad_vertical = dpad_v

        # -----------------------------
        # D-Pad Left / Right
        # -----------------------------
        dpad_h = int(msg.axes[self.dpad_horizontal])

        if dpad_h != self.prev_dpad_horizontal:

            if dpad_h == -1:

                self.angular_scale *= 1.10
                self.angular_scale = min(self.angular_scale,
                                         self.max_angular)

                self.print_speed()

            elif dpad_h == 1:

                self.angular_scale *= 0.90
                self.angular_scale = max(self.angular_scale,
                                         self.min_angular)

                self.print_speed()

        self.prev_dpad_horizontal = dpad_h

        # -----------------------------
        # Face Button Servo Control
        # Y = Pitch Up | A = Pitch Down
        # B = Yaw Right | X = Yaw Left
        # (Each press = SERVO_STEP degrees)
        # -----------------------------
        num_buttons = len(msg.buttons)

        def btn(idx):
            return int(msg.buttons[idx]) if idx < num_buttons else 0

        cur_A = btn(self.btn_A)
        cur_B = btn(self.btn_B)
        cur_X = btn(self.btn_X)
        cur_Y = btn(self.btn_Y)

        servo_changed = False

        # Y pressed (rising edge) → Pitch UP (decrease pitch angle)
        if cur_Y == 1 and self.prev_btn_Y == 0:
            new_pitch = max(self.PITCH_MIN, self.servo_pitch - self.SERVO_STEP)
            if new_pitch != self.servo_pitch:
                self.servo_pitch = new_pitch
                servo_changed = True
                self.get_logger().info(f"[Y] Pitch UP → {self.servo_pitch}°")
            else:
                self.get_logger().warn(f"[Y] Pitch UP blocked — at minimum ({self.PITCH_MIN}°)")

        # A pressed (rising edge) → Pitch DOWN (increase pitch angle)
        if cur_A == 1 and self.prev_btn_A == 0:
            new_pitch = min(self.PITCH_MAX, self.servo_pitch + self.SERVO_STEP)
            if new_pitch != self.servo_pitch:
                self.servo_pitch = new_pitch
                servo_changed = True
                self.get_logger().info(f"[A] Pitch DOWN → {self.servo_pitch}°")
            else:
                self.get_logger().warn(f"[A] Pitch DOWN blocked — at maximum ({self.PITCH_MAX}°)")

        # B pressed (rising edge) → Yaw RIGHT (towards -90°)
        if cur_B == 1 and self.prev_btn_B == 0:
            new_yaw = max(self.YAW_MIN, self.servo_yaw - self.SERVO_STEP) if self.use_sim else min(self.YAW_MAX, self.servo_yaw + self.SERVO_STEP)
            if new_yaw != self.servo_yaw:
                self.servo_yaw = new_yaw
                servo_changed = True
                self.get_logger().info(f"[B] Yaw RIGHT → {self.servo_yaw}°")
            else:
                self.get_logger().warn(f"[B] Yaw RIGHT blocked — at limit ({self.YAW_MIN if self.use_sim else self.YAW_MAX}°)")

        # X pressed (rising edge) → Yaw LEFT (towards +90°)
        if cur_X == 1 and self.prev_btn_X == 0:
            new_yaw = min(self.YAW_MAX, self.servo_yaw + self.SERVO_STEP) if self.use_sim else max(self.YAW_MIN, self.servo_yaw - self.SERVO_STEP)
            if new_yaw != self.servo_yaw:
                self.servo_yaw = new_yaw
                servo_changed = True
                self.get_logger().info(f"[X] Yaw LEFT → {self.servo_yaw}°")
            else:
                self.get_logger().warn(f"[X] Yaw LEFT blocked — at limit ({self.YAW_MAX if self.use_sim else self.YAW_MIN}°)")

        # Publish servo command only on a state change
        if servo_changed:
            self._publish_servo_cmd(self.servo_pitch, self.servo_yaw)

        # Update previous button states
        self.prev_btn_A = cur_A
        self.prev_btn_B = cur_B
        self.prev_btn_X = cur_X
        self.prev_btn_Y = cur_Y

        # -----------------------------
        # SELECT → Park | START → Default Scan
        # -----------------------------
        cur_SELECT = btn(self.btn_SELECT)
        cur_START  = btn(self.btn_START)

        # SELECT pressed (rising edge) → move to PARKED position
        if cur_SELECT == 1 and self.prev_btn_SELECT == 0:
            self.servo_yaw   = self.PARKED_YAW
            self.servo_pitch = self.PARKED_PITCH
            self._publish_servo_cmd(self.servo_pitch, self.servo_yaw)
            self.get_logger().info(
                f"[SELECT] → PARKED position: yaw={self.servo_yaw}°, pitch={self.servo_pitch}°"
            )

        # START pressed (rising edge) → move to DEFAULT SCAN position
        if cur_START == 1 and self.prev_btn_START == 0:
            self.servo_yaw   = self.DEFAULT_YAW
            self.servo_pitch = self.DEFAULT_PITCH
            self._publish_servo_cmd(self.servo_pitch, self.servo_yaw)
            self.get_logger().info(
                f"[START] → DEFAULT SCAN position: yaw={self.servo_yaw}°, pitch={self.servo_pitch}°"
            )

        self.prev_btn_SELECT = cur_SELECT
        self.prev_btn_START  = cur_START

        # -----------------------------
        # Status Shortcut Buttons 6-9
        # Each publishes a preset /bot_status message via ros2 topic pub --once
        # -----------------------------
        cur_6 = btn(self.btn_STATUS_6)
        cur_7 = btn(self.btn_STATUS_7)
        cur_8 = btn(self.btn_STATUS_8)
        cur_9 = btn(self.btn_STATUS_9)

        for cur, prev, btn_idx in [
            (cur_6, self.prev_btn_6, 6),
            (cur_7, self.prev_btn_7, 7),
            (cur_8, self.prev_btn_8, 8),
            (cur_9, self.prev_btn_9, 9),
        ]:
            if cur == 1 and prev == 0:
                status_msg = self.STATUS_MESSAGES[btn_idx]
                self.get_logger().info(
                    f"[BTN {btn_idx}] Publishing /bot_status → '{status_msg}'"
                )
                subprocess.Popen([
                    'ros2', 'topic', 'pub', '--once',
                    '/bot_status',
                    'std_msgs/msg/String',
                    f"{{data: '{status_msg}'}}"
                ])

        self.prev_btn_6 = cur_6
        self.prev_btn_7 = cur_7
        self.prev_btn_8 = cur_8
        self.prev_btn_9 = cur_9

        # -----------------------------
        # Flashlight Toggle (Right Bumper RB / R1 - button 5)
        # -----------------------------
        cur_fl = btn(self.btn_FLASHLIGHT)
        if cur_fl == 1 and self.prev_btn_FLASHLIGHT == 0:
            self.get_logger().info("[JOY] Button 5 (RB) Pressed → Toggling Flashlight")
            self.toggle_flashlight()
        self.prev_btn_FLASHLIGHT = cur_fl

        # -----------------------------
        # Publish Twist (Arcade Steering Mixing for 6-Wheel Rover)
        # -----------------------------
        deadzone = 0.05

        linear_raw = msg.axes[self.linear_axis] if len(msg.axes) > self.linear_axis else 0.0
        angular_raw = msg.axes[self.angular_axis] if len(msg.axes) > self.angular_axis else 0.0

        # Controller fallback: if angular_axis (2) is inactive, check axis 3 (Xbox / Logitech Right Stick X)
        if abs(angular_raw) < deadzone and len(msg.axes) > 3 and abs(msg.axes[3]) >= deadzone:
            angular_raw = msg.axes[3]

        if abs(linear_raw) < deadzone:
            linear_raw = 0.0
        if abs(angular_raw) < deadzone:
            angular_raw = 0.0

        twist = Twist()

        if linear_raw == 0.0 and angular_raw == 0.0:
            # Full stop
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        elif angular_raw == 0.0:
            # Pure forward or backward motion
            twist.linear.x = linear_raw * self.linear_scale
            twist.angular.z = 0.0

        elif linear_raw == 0.0:
            # Pure in-place rotation (spin left or right)
            twist.linear.x = 0.0
            twist.angular.z = angular_raw * self.angular_scale

        else:
            # Combined motion: forwardright, forwardleft, backwardright, backwardleft
            # Universal differential drive kinematic mixing:
            # Works consistently at ANY speed scale (whether 0.5 m/s or 5.0+ m/s).
            # The outer wheels run at full target speed while the inner wheels are
            # modulated down to a controlled reverse (-0.35 * v_target) at full steering
            # deflection, guaranteeing the 6-wheel rover curves decisively at any speed.
            v_target = linear_raw * self.linear_scale
            steer = angular_raw  # Right is negative (<0), Left is positive (>0)

            # Inner wheel slows down and smoothly transitions to reverse at high deflection
            inner_factor = 1.0 - 1.35 * abs(steer)

            if steer < 0.0:
                # Turning Right: left wheels are outer, right wheels are inner
                v_left = v_target
                v_right = v_target * inner_factor
            else:
                # Turning Left: right wheels are outer, left wheels are inner
                v_right = v_target
                v_left = v_target * inner_factor

            # Convert wheel linear speeds to unicycle (linear.x, angular.z)
            # Track width W = 0.62942 m
            W = 0.62942
            twist.linear.x = (v_left + v_right) / 2.0
            twist.angular.z = (v_right - v_left) / W

        self.publisher.publish(twist)

    def _publish_servo_cmd(self, pitch: int, yaw: int):
        """Publish a MOVE command to /camera_servo_cmd (JSON, same format as rack scanner).
        In simulation mode, also publish to Gazebo joint position controller topics."""
        cmd = {
            "cmd": "MOVE",
            "pitch": pitch,
            "yaw": yaw,
            "speed": self.SERVO_SPEED,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self.servo_cmd_pub.publish(msg)

        # In simulation, also publish to Gazebo joint position controllers
        if self.use_sim:
            # Yaw: 0° = forward (0 rad), -90° = right (-π/2 rad), +90° = left (+π/2 rad)
            yaw_rad = math.radians(yaw)
            yaw_msg = Float64()
            yaw_msg.data = yaw_rad
            self.gz_yaw_pub.publish(yaw_msg)

            # Pitch: 0° = level (0 rad), -90° = up (-π/2 rad), +90° = down (+π/2 rad)
            pitch_rad = math.radians(pitch)
            pitch_msg = Float64()
            pitch_msg.data = pitch_rad
            self.gz_pitch_pub.publish(pitch_msg)

        self.get_logger().info(
            f"Servo CMD → pitch={pitch}°, yaw={yaw}°, speed={self.SERVO_SPEED}"
        )

    def flashlight_callback(self, msg: Bool):
        """Subscriber callback to turn flashlight on/off via ROS2 topic."""
        self.set_flashlight(msg.data)

    def toggle_flashlight(self):
        """Toggle the camera flashlight state."""
        self.set_flashlight(not self.flashlight_on)

    def set_flashlight(self, state: bool):
        """Set the camera flashlight intensity in Gazebo or send command to ESP32."""
        self.flashlight_on = bool(state)
        state_str = "ON 💡" if self.flashlight_on else "OFF 🌑"
        self.get_logger().info(f"[FLASHLIGHT] Camera Flashlight switched → {state_str}")

        # Publish ROS 2 status
        status_msg = Bool()
        status_msg.data = self.flashlight_on
        self.flashlight_status_pub.publish(status_msg)

        if self.use_sim:
            self._send_gazebo_light_cmd(self.flashlight_on)
        else:
            # Real robot: forward command to ESP32
            cmd = {
                "cmd": "FLASHLIGHT",
                "state": self.flashlight_on
            }
            msg = String()
            msg.data = json.dumps(cmd)
            self.servo_cmd_pub.publish(msg)

    def _send_gazebo_light_cmd(self, is_on: bool):
        def _worker():
            try:
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


def main(args=None):

    rclpy.init(args=args)

    node = PS2Teleop()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()