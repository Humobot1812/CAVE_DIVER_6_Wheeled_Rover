# SIH Ground Control Station (GCS)
## 🚀 Quick Start
```bash
cd /home/abhinav/sih/src/GCS
pip install flask opencv-python numpy
python3 launch_gcs.py
# Open browser: http://localhost:5001
```

## 📁 Structure
```
GCS/
├── gcs_server.py         # Flask backend + ROS 2 bridge
├── launch_gcs.py         # Launcher script
├── captures/             # Saved snapshots
├── templates/
│   └── index.html        # Cockpit dashboard UI
└── static/
    ├── css/gcs_theme.css # OLED dark tactical theme
    └── js/gcs_app.js     # Frontend logic + Chart.js
```

## 🎮 Controls
| Key | Action |
|-----|--------|
| W/S or ↑/↓ | Forward / Backward |
| A/D or ←/→ | Turn Left / Right |
| L | Toggle Flashlight |
| E | Emergency Stop |
| C | Cycle Thermal Colormap |
| P | Save Dual Snapshot |
| H | Hazard Simulation |

## 📡 ROS 2 Topics
- **/camera/image_raw** — RGB camera input
- **/thermal/image_raw** — Thermal LWIR input
- **/cmd_vel** — Robot drive commands
- **/camera_flashlight/switch** — Flashlight control
- **/camera/gimbal_yaw** — Camera pan
- **/camera/gimbal_pitch** — Camera tilt
- **/scan** — LiDAR radar display
