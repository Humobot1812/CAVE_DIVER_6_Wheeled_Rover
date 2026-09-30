<div align="center">

<img src="docs/images/gcs_dashboard.png" alt="CAVE DIVER Ground Control Station Dashboard" width="100%"/>

# CAVE DIVER — SIH 2026

### Autonomous Subterranean Reconnaissance & Hazardous Environment Monitoring Platform

[![ROS 2](https://img.shields.io/badge/ROS_2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)
[![Ignition Gazebo](https://img.shields.io/badge/Ignition_Gazebo-Fortress_6-FF7F00)](https://gazebosim.org/)
[![Python](https://img.shields.io/badge/Python-3.10+-3776AB?logo=python)](https://www.python.org/)
[![Flask](https://img.shields.io/badge/Flask-3.1-000000?logo=flask)](https://flask.palletsprojects.com/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.12-5C3EE8?logo=opencv)](https://opencv.org/)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu_22.04_LTS-E95420?logo=ubuntu)](https://releases.ubuntu.com/22.04/)

> **Smart India Hackathon 2026** — An autonomous unmanned ground robotic system engineered for subterranean exploration, hazardous gas sensing, dual-spectrum visual reconnaissance, and web-based tactical command in GPS-denied environments.

</div>

---

## Table of Contents

- [System Overview](#system-overview)
- [System Architecture](#system-architecture)
- [Core Subsystems](#core-subsystems)
- [Ground Control Station (GCS)](#ground-control-station-gcs)
- [Dual-Spectrum Optical and Thermal System](#dual-spectrum-optical-and-thermal-system)
- [Subterranean Simulation Environment](#subterranean-simulation-environment)
- [Mapping and Localization](#mapping-and-localization)
- [Teleoperation and Control](#teleoperation-and-control)
- [System Prerequisites](#system-prerequisites)
- [Installation and Build](#installation-and-build)
- [System Execution](#system-execution)
- [ROS 2 Interface Reference](#ros-2-interface-reference)
- [Ground Control Station REST API](#ground-control-station-rest-api)
- [Project Directory Structure](#project-directory-structure)
- [QR Landmark Localization System](#qr-landmark-localization-system)
- [Project Team](#project-team)
- [License](#license)

---

## System Overview

The **CAVE DIVER** robotic platform is designed for operation in GPS-denied, structurally complex, and atmospherically hazardous subterranean environments such as mines, collapsed tunnels, and natural caverns. Developed for **Smart India Hackathon 2026**, the system integrates:

- **Dual-Spectrum Visual Inspection**: Simultaneous synchronized optical RGB and calibrated long-wave infrared (LWIR) thermal video streams with heads-up tactical diagnostics.
- **Atmospheric Hazard Monitoring**: Continuous telemetry for six critical environmental gases and parameters: Temperature, Relative Humidity, Carbon Dioxide (CO2), Oxygen (O2), Methane (CH4), and Carbon Monoxide (CO).
- **Web-Based Ground Control Station**: High-performance operator dashboard running via Flask and HTML5/WebSocket, accessible on standard field laptops without requiring local ROS installation.
- **Autonomous and Teleoperated Navigation**: Differential drive kinematics operable via dual-axis joystick controllers, automated velocity commands, or mission scripts.
- **LiDAR Sweep and Radar Display**: 2D LiDAR range sensing visualized as an active tactical sweep display for proximity assessment.
- **GPS-Free Landmark Navigation**: High-contrast encoded matrix landmarks positioned across tunnel passages for absolute metric verification in GPS-denied subterranean sectors.
- **Synchronized Spotlight Illumination**: High-intensity tactical illumination synchronized across hardware actuation and simulation rendering engines.

---

## System Architecture

```
+-------------------------------------------------------------------------+
|                           CAVE DIVER Architecture                       |
|                                                                         |
|  +--------------------+   ros_gz_bridge   +--------------------------+  |
|  |  Ignition Gazebo   |<----------------->|  Ground Control Station  |  |
|  |  simple_cave_01    |                   |  Flask + ROS 2 Node      |  |
|  |                    |  /camera/..       |  gcs_server.py           |  |
|  |  * Armo_bot URDF   |  /thermal/..      +------------+-------------+  |
|  |  * Cave SDF World  |  /scan                         |                |
|  |  * LiDAR & IMU     |  /odom                         | HTTP / MJPEG   |
|  |  * RGB & Thermal   |  /cmd_vel (<--)                v                |
|  |    Sensors         |                   +--------------------------+  |
|  +---------+----------+                   |  Operator Web Interface  |  |
|            |                              |  http://localhost:5001   |  |
|  +---------v----------+                   |                          |  |
|  |  Joystick Teleop   |                   |  * Dual Spectrum Feeds   |  |
|  |  (PS2 / Xbox)      |---> /cmd_vel      |  * Environmental Gauges  |  |
|  +--------------------+                   |  * LiDAR Radar Display   |  |
|                                           |  * Spotlight Actuation   |  |
|                                           +--------------------------+  |
+-------------------------------------------------------------------------+
```

### Communication Topology

| Origin Node | Topic / Channel | Message Type | Target Node | Role |
|-------------|-----------------|--------------|-------------|------|
| Gazebo Simulation | `/camera/image_raw` | `sensor_msgs/Image` | GCS Server | Optical RGB feed for operator surveillance |
| Gazebo Simulation | `/thermal/image_raw` | `sensor_msgs/Image` | GCS Server | LWIR thermal feed for heat signature inspection |
| Gazebo Simulation | `/scan` | `sensor_msgs/LaserScan` | GCS Server | 2D LiDAR range data for radar display |
| Gazebo Simulation | `/odom` | `nav_msgs/Odometry` | GCS Server | Metric position (X, Y) and heading (Yaw) |
| GCS / Teleop Node | `/cmd_vel` | `geometry_msgs/Twist` | Gazebo Robot | Velocity commands for vehicle drive |
| GCS Server | `/camera_flashlight/switch` | `std_msgs/Bool` | Gazebo Bridge | On/off state control for optical spotlight |

---

## Core Subsystems

| Module | Implementation | Functional Scope |
|--------|----------------|-------------------|
| `simulation` | ROS 2 / CMake | Robot mechanical model (Armo_bot), SDF subterranean world, RViz configurations, and sensor bridge bindings. |
| `joy_teleop_simulation` | ROS 2 / Python | Multi-axis joystick mapping node supporting real-time chassis motion and gimbal pan/tilt actuation. |
| `GCS` | Python / Flask | Web-based Ground Control Station server hosting telemetry polling, API endpoints, and MJPEG video streaming. |
| `scripts` | Python / OpenCV | Standalone desktop visualizer (`view_dual_camera.py`) providing high-framerate local dual-spectrum display. |
| `maps` | YAML / PGM | Occupancy grid representations generated through SLAM Toolbox for 2D subterranean localization. |

---

## Ground Control Station (GCS)

The Ground Control Station provides a browser-based operations center requiring no desktop dependencies on operator machines. The backend service bridges directly to ROS 2 topic streams and translates them into standard HTTP/MJPEG protocols.

<div align="center">
<img src="docs/images/gcs_dashboard.png" alt="Ground Control Station Live Dashboard" width="100%"/>
<br/><em>Figure 1: Operational Ground Control Station interface displaying optical and thermal video streams, environmental metrics, LiDAR radar sweep, and vehicle telemetry.</em>
</div>

### Operational Capabilities

| Capability | Specification |
|------------|---------------|
| Optical RGB Stream | Low-latency MJPEG transport at 640x480 resolution with embedded crosshair, stream diagnostics, and heartbeat indicator. |
| Thermal Infrared Stream | Radiometric representation supporting seven standard colormaps: INFERNO, JET, HOT, MAGMA, PLASMA, TURBO, and GRAYSCALE. |
| Tactical Illumination | Bidirectional control synchronized with Gazebo light properties, coupled with radial Gaussian intensity profiling in optical processing. |
| Atmospheric Monitoring | Real-time gauge metrics for Temperature, Humidity, CO2 (ppm), O2 (%), CH4 (%), and CO (ppm) with defined threshold warnings. |
| Kinematic Telemetry | Real-time Cartesian coordinate readouts (X, Y in meters), orientation heading (degrees), battery status, and latency indicators. |
| LiDAR Tactical Radar | Continuous polar sweep renderer translating planar laser scan samples into an intuitive range indicator. |
| Time-Series History | Rolling historical telemetry charts across all environmental sensor channels. |
| Mission Audit Log | Timestamped chronological log recording operational commands, state changes, and alert triggers. |
| Frame Capture Utility | Immediate archival of current camera frame buffers to the local file system. |
| Independent Demo Mode | Built-in synthetic signal generator enabling complete UI verification and demonstration without an active ROS core. |

### Station Quick Start

```bash
# 1. Install prerequisites
pip install flask opencv-python numpy

# 2. Launch the server
cd GCS
python3 launch_gcs.py

# 3. Access the operations console
# Navigate to: http://localhost:5001
```

**Binding to specific host or network interfaces:**
```bash
python3 launch_gcs.py --host 0.0.0.0 --port 8080
```

---

## Dual-Spectrum Optical and Thermal System

Visual inspection in dark, dusty, or unventilated tunnels demands complementary optical and infrared sensors. The CAVE DIVER vision pipeline runs simultaneously across two independent sensor channels.

### Heads-Up Display (HUD) Specifications

**RGB Optical Stream Diagnostics:**
- Active heartbeat state (`LIVE`)
- Stream resolution and processing framerate (`640x480 @ 20.0 FPS`)
- Primary crosshair reticle for vehicle heading alignment
- Spotlight operational status (`FLASHLIGHT: ACTIVE / INACTIVE`)

**LWIR Thermal Stream Diagnostics:**
- Active colormap descriptor and framerate
- Central spot temperature readout (`SPOT: XX.X C`)
- Dynamic scene metrics: Minimum, Maximum, and Dynamic Thermal Range
- Extremum targeting: Dedicated red reticle on peak thermal source, yellow reticle on minimum thermal point

### Standalone Desktop Viewer

For direct high-performance inspection on workstation monitors:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 scripts/view_dual_camera.py
```

**Viewer Keybindings:**

| Key | Operation |
|:---:|-----------|
| `L` | Toggle illumination spotlight |
| `C` | Cycle thermal colormap palette |
| `S` | Archive optical and thermal snapshot |
| `W` | Toggle between unified split display and individual windows |
| `Q` / `ESC` | Terminate viewer process |

### Spotlight Beam Algorithm

The tactical spotlight applies a localized radial Gaussian intensity distribution across color channels to model physical beam dispersion:

```python
boosted[:, :, 0] *= (1.0 + beam * 1.2)  # Blue spectrum amplification
boosted[:, :, 1] *= (1.0 + beam * 1.4)  # Green spectrum amplification
boosted[:, :, 2] *= (1.0 + beam * 1.5)  # Red spectrum amplification
```

---

## Subterranean Simulation Environment

High-fidelity physics and sensor modeling are executed in Ignition Gazebo Fortress within a dedicated subterranean cave SDF world.

<div align="center">

| Robot Perspective — Wide Field | Robot Perspective — Sensor Frustum |
|:------------------------------:|:----------------------------------:|
| <img src="docs/images/robot_cave_wide.png" alt="Armo_bot platform wide view in subterranean cave" width="100%"/> | <img src="docs/images/robot_cave_close.png" alt="Armo_bot platform close view with camera frustum" width="100%"/> |

| Robot Perspective — Orthographic | Subterranean Occupancy Grid (RViz) |
|:--------------------------------:|:----------------------------------:|
| <img src="docs/images/robot_cave_top.png" alt="Top-down orthographic view of robot in cave" width="100%"/> | <img src="docs/images/cave_map_rviz.png" alt="Occupancy grid generated in RViz" width="100%"/> |

</div>

### World Definition: `simple_cave_01.sdf`

- Geometry: Irregular, non-uniform subterranean tunnel walls and rocky passage constraints.
- Topography: Winding bifurcated paths simulating real underground mining galleries.
- Landmarks: High-resolution QR position markers positioned along interior tunnel surfaces.
- Atmospheric Lighting: Dark ambient settings designed for sensor illumination stress testing.

### Robot Platform: `Armo_bot`

Defined within `simulation/urdf_sih/` using modular URDF/Xacro descriptions:

| File | Subsystem Description |
|------|-----------------------|
| `robot_base.urdf.xacro` | Top-level assembly integrating kinematics, sensor joints, and visual meshes. |
| `base_mobile.xacro` | Differential drive chassis layout, wheel physical parameters, and collision volumes. |
| `common_properties.xacro` | Inertial matrices, mass constants, and shared material properties. |
| `robot_gazebo.xacro` | Ignition Gazebo sensor plugins: differential drive controller, IMU, planar LiDAR, cameras, and spotlight. |

---

## Mapping and Localization

<div align="center">
<img src="docs/images/cave_map_rviz.png" alt="Subterranean Cave Occupancy Grid" width="70%"/>
<br/><em>Figure 2: 2D Occupancy grid map of the subterranean world generated via SLAM Toolbox.</em>
</div>

The repository includes a calibrated 2D occupancy grid map (`maps/Cave_map.pgm` and `maps/Cave_map.yaml`) of the simulated cave:

| Parameter | Calibrated Value | Unit |
|-----------|------------------|------|
| Spatial Resolution | `0.05` | meters / pixel |
| Map Origin | `[-88.8, -22.5, 0.0]` | meters, meters, radians |
| Occupied Cell Threshold | `0.65` | probability ratio |
| Free Cell Threshold | `0.25` | probability ratio |
| Representation Mode | `Trinary` | categorical |

---

## Teleoperation and Control

### Joystick Teleoperation Node

```bash
ros2 launch joy_teleop_simulation teleop_sim.launch.py
```

**Gamepad Controller Input Mapping:**

| Function | Primary Input | Hardware Index |
|----------|---------------|----------------|
| Longitudinal Drive (Forward / Backward) | Left Analog Stick (Vertical) | Axis 1 |
| Angular Rotation (Left / Right) | Right Analog Stick (Horizontal) | Axis 2 |
| Camera Gimbal Pan Left | Action Button X | Button 3 |
| Camera Gimbal Pan Right | Action Button B | Button 1 |
| Camera Gimbal Tilt Up | Action Button Y | Button 4 |
| Camera Gimbal Tilt Down | Action Button A | Button 0 |
| Spotlight Actuation Toggle | Select Button | Button 8 |
| Emergency Safe Stop | Start Button | Button 9 |

### Deployment Modes

```bash
# Simulation Environment (Gazebo clock integration)
ros2 launch joy_teleop_simulation teleop_sim.launch.py

# Physical Robotic Hardware
ros2 run joy_teleop_simulation teleop_node --ros-args -p use_sim:=false
```

---

## System Prerequisites

### Operating Environment
- Operating System: Ubuntu 22.04 LTS (Jammy Jellyfish)
- System Memory: 8 GB minimum, 16 GB recommended
- Graphics Hardware: Dedicated GPU recommended for physics simulation and ray tracing

### Required Software Packages

```bash
# ROS 2 Humble base installation
sudo apt install ros-humble-desktop

# Ignition Gazebo 6 (Fortress)
sudo apt install ignition-fortress

# ROS-Gazebo transport bridges
sudo apt install ros-humble-ros-gz-bridge ros-humble-ros-gz-sim

# Mapping and Navigation components
sudo apt install ros-humble-slam-toolbox ros-humble-navigation2 ros-humble-nav2-bringup

# Vision and Image Transport pipelines
sudo apt install ros-humble-cv-bridge ros-humble-image-transport

# Gamepad joystick interface
sudo apt install ros-humble-joy

# Ground Control Station Python dependencies
pip install flask opencv-python numpy
```

---

## Installation and Build

```bash
mkdir -p ~/CAVE_DIVER
cd ~/CAVE_DIVER
# 1. Clone the repository
git clone https://github.com/Humobot1812/CAVE_DIVER_6_Wheeled_Rover.git
mv CAVE_DIVER_6_Wheeled_Rover src


# 2. Resolve and install system dependencies
rosdep install --from-paths . --ignore-src -r -y

# 3. Build workspace packages
colcon build --symlink-install

# 4. Source the built environment
source install/setup.bash
```

---

## System Execution

### Standard Full-Stack Startup Sequence

**Terminal 1 — Ignition Gazebo Subterranean World:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch simulation test_cave.launch.xml
```

**Terminal 2 — ROS-Gazebo Communication Bridge:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 run ros_gz_bridge parameter_bridge \
  --ros-args -p config_file:=simulation/config/gazebo_bridge.yaml
```

**Terminal 3 — Joystick Teleoperation:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch joy_teleop_simulation teleop_sim.launch.py
```

**Terminal 4 — Ground Control Station (GCS):**
```bash
cd GCS
python3 launch_gcs.py
# Access console at http://localhost:5001
```

**Optional — Standalone Desktop Camera Visualizer:**
```bash
source /opt/ros/humble/setup.bash && source install/setup.bash
python3 scripts/view_dual_camera.py
```

---

## ROS 2 Interface Reference

| Topic Path | ROS 2 Message Type | Flow Direction | Description |
|------------|-------------------|----------------|-------------|
| `/camera/image_raw` | `sensor_msgs/Image` | Gazebo -> ROS | Optical RGB camera frame stream |
| `/thermal/image_raw` | `sensor_msgs/Image` | Gazebo -> ROS | Long-wave infrared thermal image stream |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo -> ROS | Planar LiDAR range scan array |
| `/scan/points` | `sensor_msgs/PointCloud2` | Gazebo -> ROS | Unorganized 3D point cloud array |
| `/imu` | `sensor_msgs/Imu` | Gazebo -> ROS | Inertial measurement unit data |
| `/odom` | `nav_msgs/Odometry` | Gazebo -> ROS | Odometric state estimate (pose and twist) |
| `/tf` | `tf2_msgs/TFMessage` | Gazebo -> ROS | Coordinate frame transformation tree |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS -> Gazebo | Chassis linear and angular velocity commands |
| `/camera_flashlight/switch` | `std_msgs/Bool` | ROS -> Gazebo | Boolean command to actuate spotlight state |
| `/camera_flashlight/status` | `std_msgs/Bool` | Gazebo -> ROS | Confirmation feedback of spotlight state |
| `/camera_base_cylinder/cmd_pos` | `std_msgs/Float64` | ROS -> Gazebo | Gimbal pan joint position command (rad) |
| `/camera_cylinder_sphere/cmd_pos` | `std_msgs/Float64` | ROS -> Gazebo | Gimbal tilt joint position command (rad) |
| `/camera/gimbal_yaw` | `std_msgs/Float32` | ROS Internal | Logical camera yaw command |
| `/camera/gimbal_pitch` | `std_msgs/Float32` | ROS Internal | Logical camera pitch command |

---

## Ground Control Station REST API

The GCS server exposes a clean REST interface for integration with higher-level dispatch and logging systems.

| Endpoint | HTTP Method | Payload / Response | Functional Description |
|----------|:-----------:|-------------------|------------------------|
| `/` | `GET` | HTML Document | Serves the main browser dashboard |
| `/stream/rgb` | `GET` | `multipart/x-mixed-replace` | Live optical RGB MJPEG video stream |
| `/stream/thermal` | `GET` | `multipart/x-mixed-replace` | Live thermal infrared MJPEG video stream |
| `/api/telemetry` | `GET` | JSON Object | Returns position, battery, gas levels, and scan ranges |
| `/api/flashlight` | `POST` | `{"state": boolean}` | Commands tactical spotlight state |
| `/api/move` | `POST` | `{"linear": float, "angular": float}` | Dispatches velocity vector to `/cmd_vel` |
| `/api/colormap` | `POST` | `{"name": string}` | Configures active thermal false-color palette |
| `/api/snapshot` | `POST` | JSON Status | Captures and persists the active camera frame buffer |
| `/api/log` | `GET` | JSON Array | Retrieves chronological mission log events |

**Sample API Command:**
```bash
curl -X POST http://localhost:5001/api/flashlight \
  -H "Content-Type: application/json" \
  -d '{"state": true}'
```

---

## Project Directory Structure

```
CAVE_DIVER_6_Wheeled_Rover/
|-- .gitignore                      # Git exclusion rules
|-- README.md                       # Master system documentation
|-- docs/
|   `-- images/
|       |-- gcs_dashboard.png        # Operational dashboard capture
|       |-- robot_cave_wide.png      # Robot wide perspective in simulation
|       |-- robot_cave_close.png     # Close perspective with sensor frustum
|       |-- robot_cave_top.png       # Top-down orthographic perspective
|       `-- cave_map_rviz.png        # SLAM occupancy grid visualization
|
|-- GCS/                            # Ground Control Station subsystem
|   |-- gcs_server.py               # Flask application & ROS 2 interface node
|   |-- launch_gcs.py               # Environment validator and service launcher
|   |-- captures/                   # Snapshot archival storage
|   |-- templates/index.html        # Operations dashboard HTML
|   `-- static/
|       |-- css/gcs_theme.css       # Tactical dark theme design system
|       `-- js/gcs_app.js           # Telemetry synchronization and UI controller
|
|-- simulation/                     # Physics simulation subsystem
|   |-- package.xml                 # ROS 2 package descriptor
|   |-- CMakeLists.txt              # CMake build configuration
|   |-- config/
|   |   |-- gazebo_bridge.yaml      # ROS 2 <-> Gazebo Fortress topic mappings
|   |   `-- Slam_param.yaml         # SLAM Toolbox tuning configuration
|   |-- launch/
|   |   |-- test_cave.launch.xml    # Primary subterranean simulation bringup
|   |   |-- display.launch.xml      # Robot model inspection
|   |   `-- display_sih_map.launch.xml # Map visualization in RViz
|   |-- urdf_sih/
|   |   |-- robot_base.urdf.xacro   # Primary robot assembly
|   |   |-- base_mobile.xacro       # Kinematic platform description
|   |   |-- common_properties.xacro # Physical materials and inertias
|   |   `-- robot_gazebo.xacro      # Gazebo plugin attachments
|   |-- rviz/rviz_config.rviz       # Calibrated RViz workspace layout
|   `-- world/
|       |-- simple_cave_01.sdf      # Subterranean cave environment definition
|       `-- materials/
|           |-- meshes/             # Landmark geometric models (STL)
|           `-- textures/           # High-contrast QR section textures
|
|-- joy_teleop_simulation/          # Teleoperation subsystem
|   |-- package.xml                 # Python package descriptor
|   |-- setup.py                    # Package build script
|   |-- joy_teleop_simulation/
|   |   `-- teleop_node.py          # Controller event processing node
|   `-- launch/
|       `-- teleop_sim.launch.py    # Teleoperation launch configuration
|
|-- maps/                           # Environmental mapping data
|   |-- Cave_map.pgm                # 2D occupancy grid raster
|   `-- Cave_map.yaml               # Metric calibration and map metadata
|
`-- scripts/
    `-- view_dual_camera.py         # High-framerate desktop camera visualizer
```

---

## QR Landmark Localization System

Over 60 high-contrast QR code matrices are embedded throughout the subterranean tunnel passages to provide reliable, drift-free positional verification in GPS-denied zones:

```
Encoding Specification: WH-{Area}-{Room}-{Rack}-{Section}-{Position}
Example Landmark Code : WH-A1-R1-RK1-S1-P1
```

Each marker uniquely identifies a subterranean coordinate block. When detected by the optical pipeline, these identifiers offer absolute metric ground truth to constrain odometric drift and update the global mission state.

---

## Project Team

**Project: CAVE DIVER**  
**Initiative: Smart India Hackathon 2026**

| Role | Contributor |
|------|-------------|
| Platform Engineering, GCS Architecture, Simulation & Control | Abhinav |

- Contact: ironman18122004@gmail.com

---

## License

Developed for Research . All rights reserved.

---

<div align="center">


*ROS 2 Humble · Ignition Gazebo Fortress · OpenCV · Flask · Python 3.10*

</div>
