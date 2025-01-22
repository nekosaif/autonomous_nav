# Autonomous Navigation with YOLO Object Detection and Obstacle Avoidance

![ROS Version](https://img.shields.io/badge/ROS-Noetic%20%7C%20Humble-brightgreen)
![Python Version](https://img.shields.io/badge/Python-3.6%2B-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-orange)

A ROS-based autonomous navigation system integrating real-time object detection (YOLOv8) and reactive obstacle avoidance.

## 📌 Overview

This project enables robots to:
- Perform real-time object detection using YOLOv8
- Navigate autonomously while avoiding obstacles
- Track specific targets using visual recognition
- Integrate with ROS navigation stack (move_base)
- Operate in dynamic environments with configurable safety parameters

## 🚀 Key Features

- **YOLOv8 Integration**: State-of-the-art object detection
- **Obstacle Avoidance**: Laser-based reactive navigation
- **Multi-Strategy Navigation**: Combines object tracking & path planning
- **ROS Integration**: Standard message interfaces
- **Configurable Parameters**: Adjustable safety margins and detection thresholds
- **Multi-Robot Support**: Compatible with TurtleBot3, Jackal, and custom platforms

## 📦 Installation

### Prerequisites
- ROS Noetic (Ubuntu 20.04) or ROS Humble (Ubuntu 22.04)
- Python 3.6+
- NVIDIA GPU (Recommended for YOLO inference)

```bash
# Clone repository
git clone https://github.com/yourusername/autonomous_nav.git
cd autonomous_nav/ros_ws/src

# Install dependencies
pip3 install -r requirements.txt
sudo apt-get install ros-$ROS_DISTRO-cv-bridge ros-$ROS_DISTRO-move-base

# Build package
catkin_make
source devel/setup.bash
```

## 🛠️ Usage

### Launch System
```bash
# Start object detection
roslaunch autonomous_nav object_detection.launch

# Launch navigation stack
roslaunch autonomous_nav navigation.launch
```

### Key Commands
```bash
# Enable autonomous mode
rosservice call /navigation_controller/start_navigation

# View detection results (new terminal)
rqt_image_view /detection_overlay
```

## 📂 Package Structure

```bash
autonomous_nav/
├── config/
│   └── yolo_params.yaml       # Configuration parameters
├── launch/
│   ├── navigation.launch      # Navigation stack launcher
│   └── object_detection.launch# Camera & YOLO launcher
├── models/
│   └── yolov8n.pt             # YOLOv8 pretrained weights
├── scripts/
│   ├── yolo_detection_node.py # Object detection node
│   ├── obstacle_avoidance_node.py # Collision avoidance
│   └── navigation_manager.py  # Goal management
└── README.md
```

## 🖥️ Nodes Overview

### 1. YOLO Detection Node
- **Input**: RGB Camera feed (`/camera/image_raw`)
- **Output**: Object detections (`/detected_objects`)
- **Features**:
  - Real-time object detection
  - Confidence threshold filtering
  - Target class selection

### 2. Obstacle Avoidance Node
- **Input**: 
  - Laser scans (`/scan`)
  - Object detections (`/detected_objects`)
- **Output**: Velocity commands (`/cmd_vel`)
- **Features**:
  - Dynamic obstacle avoidance
  - Object tracking
  - Emergency stop functionality

### 3. Navigation Manager
- **Integration**: ROS navigation stack
- **Features**:
  - Goal management
  - Path recovery
  - Multi-object tracking

## ⚙️ Customization

Modify `config/yolo_params.yaml`:

```yaml
yolo:
  target_classes: ["person", "dog"] # Objects to track
  confidence_threshold: 0.65        # Detection confidence

navigation:
  safe_distance: 0.6                # Minimum obstacle distance (meters)
  linear_speed: 0.3                 # Base movement speed
```

## 🚨 Troubleshooting

| Issue | Solution |
|-------|----------|
| Camera not detected | Check USB permissions: `ls -l /dev/video*` |
| Low detection FPS | Enable GPU acceleration in YOLO params |
| Navigation failures | Adjust `safe_distance` in configuration |

## 🤝 Contributing

1. Fork the repository
2. Create feature branch: `git checkout -b new-feature`
3. Commit changes: `git commit -am 'Add feature'`
4. Push branch: `git push origin new-feature`
5. Submit pull request

## 📜 License

Apache 2.0 License - See [LICENSE](LICENSE) for details.

## 🙏 Acknowledgments

- Ultralytics for YOLOv8 implementation
- ROS development community
- TurtleBot3 for reference platform implementation