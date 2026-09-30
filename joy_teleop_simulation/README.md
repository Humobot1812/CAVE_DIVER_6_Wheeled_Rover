# `joy_teleop_simulation` — ROS2 Gamepad & Joystick Teleoperation

**Deployment Target**: 🖥️ **Simulation & Local Operator PC**

`joy_teleop_simulation` is a ROS2 Python (`ament_python`) package that maps USB/Bluetooth gamepads (Xbox, PlayStation, Logitech, or generic controllers) to:
1. **Robot Base Drive Commands**: `geometry_msgs/msg/Twist` published to `/diff_cont/cmd_vel_unstamped`.
2. **Camera Gimbal Control**: Joint commands for pitch tilt and yaw pan in simulation (`/camera_base_cylinder/cmd_pos` and `/camera_cylinder_sphere/cmd_pos`) or hardware ESP32 JSON bridge messages.
3. **Status Triggers**: Button triggers for initiating rack scanning routines (`/bot_status`).

---

## 📁 Package Structure

```text
joy_teleop_simulation/
├── package.xml                        # ROS2 package manifest & dependencies
├── setup.py                            # Python setup script (ament_python)
├── setup.cfg                           # Script installation directory configs
├── README.md                           # Documentation
├── resource/
│   └── joy_teleop_simulation          # Ament resource index marker
├── joy_teleop_simulation/             # Python source code
│   ├── __init__.py
│   └── teleop_node.py                 # Teleop mapper node
├── launch/                             # Launch configurations
│   └── teleop_sim.launch.py           # Launch joy_node + ps2_teleop
└── test/                               # Package test scripts
```

---

## 🛠️ Requirements & Setup

Install the official ROS2 `joy` driver package on your host machine:

```bash
sudo apt update
sudo apt install -y ros-humble-joy
```

Connect your joystick via USB or Bluetooth and verify device connection:
```bash
ls -l /dev/input/js*
```

---

## 🎮 Gamepad Mapping & Control Scheme

### 🚗 Differential Drive Velocity Controls

| Controller Input | Action | Topic / Output | Notes |
|---|---|---|---|
| **Left Stick (Vertical)** | Linear velocity ($v_x$) | `/diff_cont/cmd_vel_unstamped` | Forward / Backward movement |
| **Right Stick (Horizontal)** | Angular velocity ($\omega_z$) | `/diff_cont/cmd_vel_unstamped` | Left / Right rotation |
| **D-Pad Up / Down** | Adjust Max Linear Speed | Internal speed multiplier | Increment / decrement by 10% |
| **D-Pad Left / Right** | Adjust Max Angular Speed | Internal speed multiplier | Increment / decrement by 10% |

---

### 📷 Camera Servo & Gimbal Controls

| Button | Index | Action | Limits / Axis |
|---|---|---|---|
| **Y Button** | `4` | Tilt Camera **UP** | Pitch − 5° (Min: 20°) |
| **A Button** | `0` | Tilt Camera **DOWN** | Pitch + 5° (Max: 160°) |
| **B Button** | `1` | Pan Camera **RIGHT** | Yaw + 5° (Max: 180°) |
| **X Button** | `3` | Pan Camera **LEFT** | Yaw − 5° (Min: 0°) |
| **SELECT** | `10` | Preset **Parked Position** | Yaw: 100°, Pitch: 90° |
| **START** | `11` | Preset **Scan Position** | Yaw: 10°, Pitch: 90° |

---

### 📍 Autonomous Rack Target Triggers

| Button | Index | Published Message to `/bot_status` | Description |
|---|---|---|---|
| **L1 / LB** | `6` | `"Reached Aisle_1/Row_1/Rack_1"` | Trigger rack scan sequence for Aisle 1 |
| **R1 / RB** | `7` | `"Reached Aisle_2/Row_1/Rack_1"` | Trigger rack scan sequence for Aisle 2 |
| **L2 / LT** | `8` | `"Reached Aisle_1/Row_1/Rack_2"` | Trigger rack scan sequence for Rack 2 |
| **R2 / RT** | `9` | `"Reached Aisle_2/Row_1/Rack_2"` | Trigger rack scan sequence for Rack 2 |

---

## 📡 Published & Subscribed Topics

| Topic | Message Type | Direction | Description |
|---|---|---|---|
| `/joy` | `sensor_msgs/msg/Joy` | **Subscribed** | Raw axis and button array from `joy_node` |
| `/diff_cont/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | **Published** | Robot differential drive movement commands |
| `/camera_base_cylinder/cmd_pos` | `std_msgs/msg/Float64` | **Published** | Simulated camera Yaw joint angle command |
| `/camera_cylinder_sphere/cmd_pos` | `std_msgs/msg/Float64` | **Published** | Simulated camera Pitch joint angle command |
| `/bot_status` | `std_msgs/msg/String` | **Published** | Trigger rack scanning status events |

---

## 🚀 Usage & Launch Instructions

### 1. Build Package
```bash
cd ~/WareOps
colcon build --packages-select joy_teleop_simulation
source install/setup.bash
```

### 2. Launch Teleoperation Stack
Launch both the `joy_node` hardware driver and `teleop_node` together:
```bash
ros2 launch joy_teleop_simulation teleop_sim.launch.py
```

### 3. Run Standalone Node
```bash
ros2 run joy_teleop_simulation teleop_node --ros-args -p use_sim:=true
```
