# ADRC
Autnomous Driving Remote Control Operations.

# TODO 
# 🚗 ROS2 Remote Patrol Car with Jetson Nano & NVIDIA DeepStream

## ✅ Overview
This roadmap guides you through building an **Edge AI Remote Patrol Car** using:
- **Jetson Nano** for on-device AI
- **ROS2** for robotics middleware
- **NVIDIA DeepStream + TensorRT** for optimized inference
- **LiDAR + Camera** for mapping and perception

---

## 🛠 Prerequisites
- Jetson Nano (4GB recommended)
- MicroSD card (64GB or larger)
- Power supply (5V 4A)
- Ubuntu 18.04/20.04 PC for remote development
- Basic knowledge of Linux, ROS2, and Python

---

## 📦 Hardware Setup
1. **Chassis & Motors**
   - RC car chassis with 2 DC motors and steering servo.
   - Motor driver (e.g., TB6612FNG).

2. **Sensors**
   - Camera: CSI (IMX219) or USB UVC.
   - LiDAR: RPLIDAR A1/A2 for mapping.

3. **Networking**
   - Wi-Fi module or Ethernet for remote control.

---

## 🔍 Software Installation
### On Jetson Nano:
1. Flash **JetPack 4.x** image.
2. Enable performance mode:
   ```bash
   sudo nvpmodel -m 0
   sudo jetson_clocks
   ```
3. Add swap for stability:
   ```bash
   sudo fallocate -l 4G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

### Install ROS2 Humble/Foxy:
```bash
sudo apt update && sudo apt install ros-humble-desktop
```

### Install NVIDIA DeepStream SDK:
```bash
wget https://developer.nvidia.com/deepstream-sdk-download
sudo apt install deepstream-6.0
```

---

## 🧩 ROS2 Integration
1. Create a ROS2 workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
```

2. Add nodes:
- **Teleop Node**: `teleop_twist_keyboard` or joystick.
- **Motor Control Node**: PWM signals to motor driver.
- **LiDAR Node**: RPLIDAR driver.
- **SLAM Node**: Cartographer or RTAB-Map.

---

## 🤖 DeepStream AI Setup
1. Pull pre-trained model from NVIDIA NGC (e.g., PeopleNet):
```bash
wget https://ngc.nvidia.com/models/nvidia/peoplenet
```

2. Configure DeepStream pipeline:
- Camera input → TensorRT inference → ROS2 topic `/detections`.

3. Use GStreamer for video streaming:
```bash
gst-launch-1.0 nvarguscamerasrc ! nvv4l2h264enc ! rtph264pay ! udpsink host=<IP> port=5000
```

---

## 🕹 Teleoperation
- Use ROS2 `teleop_twist_keyboard`:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

- Or joystick/gamepad via ROS2 Joy package.

---

## 🗺 Mapping
- Launch Cartographer for SLAM:
```bash
ros2 launch cartographer_ros cartographer.launch.py
```

- Visualize map in RViz:
```bash
ros2 run rviz2 rviz2
```

---

## 📡 Streaming & Dashboard
- Optional: WebRTC for browser-based control.
- Or use GStreamer RTP for low-latency video.

---

## ✅ Testing & Optimization
- Validate teleop and mapping.
- Optimize TensorRT inference for Nano.
- Add safety features (watchdog, emergency stop).

---

## ⏳ Estimated Timeline
- Setup: 2 days
- Hardware assembly: 2 days
- ROS2 integration: 3 days
- DeepStream AI: 2 days
- Streaming & testing: 2 days
**Total: ~10–12 days**

---

## 🌐 Future Enhancements
- Autonomous navigation with waypoints.
- Multi-camera support.
- Integration with Edge AI analytics.
