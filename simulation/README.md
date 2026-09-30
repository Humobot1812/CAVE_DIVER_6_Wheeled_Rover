# `Simulation` — ROS2 Gazebo & Nav2 Simulation Package

**Deployment Target**: 🖥️ **Gazebo & RViz Simulation**

The `Simulation` package provides the core Gazebo warehouse environment, robot URDF/Xacro definitions (differential drive bot with 2-DOF active camera mast), Nav2 costmap & planner configurations, map assets, and simulation launch files for the **WareOps Autonomous Warehouse Robot System**.

---

## 📁 Package Architecture & Structure

```text
Simulation/
├── CMakeLists.txt              # Build configuration (ament_cmake)
├── package.xml                 # ROS2 package manifest & dependencies
├── README.md                   # Package documentation
├── config/                     # Navigation, SLAM & Gazebo parameters
│   ├── gazebo_bridge.yaml      # ROS2 ↔ Gazebo Topic Bridge configuration
│   ├── param_nav2.yaml         # Nav2 costmaps, AMCL, planner & controller params
│   └── Slam_param.yaml         # SLAM Toolbox mapping parameters
├── launch/                     # Simulation launch files
│   ├── display.launch.xml              # Launches Gazebo world + robot URDF spawn
│   ├── display_map.launch.xml          # Launches Gazebo world + Nav2 map server
│   └── display_navigation.launch.xml   # Launches full Gazebo + Nav2 AMCL navigation
├── maps/                       # Occupancy grid maps of the warehouse
│   ├── New_map.pgm             # Warehouse layout map grid
│   └── New_map.yaml            # Map metadata (resolution, origin, threshold)
├── rviz/                       # RViz2 visualization display profiles
│   ├── rviz_config.rviz        # Base robot model viewer config
│   └── rviz_config_nav2.rviz   # Nav2 navigation & costmap visualization profile
├── urdf/                       # Robot URDF & Xacro model descriptions
│   ├── base_mobile.xacro       # Chassis, wheels, casters, laser scanner & active vision mast
│   ├── common_properties.xacro # Inertials, colors, material definitions
│   ├── robot_base.urdf.xacro   # Entry URDF loader file
│   └── robot_gazebo.xacro      # Gazebo plugins (diff drive, joint state publisher, camera sensor)
└── world/                      # Gazebo world models & assets
    ├── warehouse.sdf           # Multi-aisle warehouse environment with QR-tagged racks
    ├── multi_camera_viewer.py  # Utility for multi-camera view rendering
    └── materials/              # Custom textures and materials for warehouse racks
```

> **Note**: System-wide multi-package launch orchestration is centralized in [`wareops_system_launch`](../wareops_system_launch).

---

## ⚙️ Core Components

### 1. Robot Model (URDF / Xacro)
- **Chassis**: Differential drive mobile platform equipped with dual drive wheels and front/rear passive casters.
- **Sensors**: 
  - 2D LiDAR (`/scan`) mounted on the robot chassis for SLAM and AMCL localization.
  - Active Vision Camera Mounted on a 2-DOF motorized mast (Yaw rotation + Pitch tilt).
- **Joint Position Control Topics**:
  - `/camera_base_cylinder/cmd_pos` (`std_msgs/Float64`): Rotates camera mast (Yaw: 0.0 to 3.14 rad).
  - `/camera_cylinder_sphere/cmd_pos` (`std_msgs/Float64`): Tilts camera lens (Pitch: 0.0 to 1.57 rad).

### 2. World & Map Environment
- **Warehouse World (`warehouse.sdf`)**: Includes high-density inventory racks, aisle pathways, walls, and realistic lighting.
- **Occupancy Grid (`New_map.yaml / New_map.pgm`)**: Calibrated 2D grid map used by Nav2 AMCL localization and costmap cost layers.

---

## 🚀 Launch Configurations

| Launch File | Command | Description |
|---|---|---|
| **Navigation & Map** | `ros2 launch Simulation display_navigation.launch.xml` | Launches Gazebo world, robot spawn, Map Server (`New_map.pgm`), AMCL localization, Nav2 planners & RViz2. |
| **Map Only** | `ros2 launch Simulation display_map.launch.xml` | Launches Gazebo and Map Server for testing map alignment. |
| **Robot Display** | `ros2 launch Simulation display.launch.xml` | Spawns robot model in Gazebo world without Nav2 navigation stack. |

> To launch the entire system (Simulation + Nav2 + Teleop + Vision Scanner + Dashboard), use:
> ```bash
> ros2 launch wareops_system_launch full_system.launch.py
> ```

---

## 📡 Published & Subscribed ROS2 Topics

| Topic | Message Type | Direction | Description |
|---|---|---|---|
| `/cmd_vel` / `/diff_cont/cmd_vel_unstamped` | `geometry_msgs/msg/Twist` | Subscribe | Robot movement velocity commands |
| `/scan` | `sensor_msgs/msg/LaserScan` | Publish | 2D LiDAR distance scanning data |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Publish | Simulated RGB camera feed from camera mast |
| `/camera_base_cylinder/cmd_pos` | `std_msgs/msg/Float64` | Subscribe | Gazebo camera Yaw joint position controller |
| `/camera_cylinder_sphere/cmd_pos` | `std_msgs/msg/Float64` | Subscribe | Gazebo camera Pitch joint position controller |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Subscribe | Initial pose estimation for Nav2 AMCL |
| `/amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Publish | Estimated robot position from AMCL filter |
| `/map` | `nav_msgs/msg/OccupancyGrid` | Publish | Static warehouse occupancy grid map |

---

## 🛠️ Build & Quick Start Instructions

1. **Build Package**:
   ```bash
   cd ~/WareOps
   colcon build --packages-select Simulation
   source install/setup.bash
   ```

2. **Launch Gazebo Simulation with Nav2**:
   ```bash
   ros2 launch Simulation display_navigation.launch.xml
   ```

3. **Launch Complete Multi-Package System**:
   ```bash
   ros2 launch wareops_system_launch full_system.launch.py
   ```
