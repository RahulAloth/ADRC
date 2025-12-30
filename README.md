# 🚗 ADRC — Autonomous Driving Remote Control

ADRC stands for **Autonomous Driving Remote Control**, a system designed to remotely manage autonomous‑driving (AD) functionality using Edge AI. This project targets the testing and validation phase of **Level‑4 and Level‑5** autonomous vehicles, where safety, cost, and operational efficiency are critical.

---

## 🚀 Project Aim

The ADRC project enables **remote control and supervision** of autonomous‑driving functions during high‑level AD testing.  
In current OEM testing setups, a safety operator must sit inside the vehicle to:

- Manually enable or disable AD functions  
- Monitor the surroundings  
- Ensure AD activation only within validated test routes  
- Verify compliance with the Operational Design Domain (ODD)

This approach is safe but **not scalable** and **not cost‑efficient**.

ADRC replaces this in‑vehicle operator with a **remote operator**, improving safety, efficiency, and scalability.

---

## 🧠 System Overview

A **Jetson device** is integrated with the vehicle’s autonomous‑driving stack. It performs two major functions:

### 1. **Remote AD Control**
- Communicates with a remote control center  
- Allows AD‑enable/disable actions from a secure external location  
- Ensures AD activation only when conditions are safe and validated  

### 2. **Edge AI Perception**
- Uses a single camera to observe the road environment  
- Evaluates the situation similar to a human operator  
- Supports decision‑making during AD engagement  
- Uses GNSS data for precise vehicle positioning  

This ensures that AD‑ready mode is activated **only on predefined, validated test paths**.

---

## 💡 Why ADRC?

### ✔ Safety  
Testing Level‑4/Level‑5 systems without exposing passengers, pedestrians, or traffic to unnecessary risk.

### ✔ Cost Efficiency  
Eliminates the need for **two people** inside the vehicle (driver + operator).  
This significantly reduces testing costs across large fleets.

### ✔ Scalability  
Remote operators can supervise **multiple vehicles**, enabling large‑scale AD validation.

### ✔ Future‑Ready  
As the system matures, the remote operator’s role can be gradually reduced and eventually replaced by **Edge‑AI‑based decision logic**, enabling:

- Autonomous AD‑readiness checks  
- Automated ODD validation  
- Fully remote, AI‑assisted AD testing workflows  

---

## 🛠 Key Components

- **NVIDIA Jetson (Xavier/Orin)**  
- **Edge AI perception pipeline**  
- **Single‑camera road‑scene understanding**  
- **GNSS‑based ODD validation**  
- **Remote operator interface**  
- **Secure communication link to AD stack**  

---

## 🧭 Testing Workflow

1. Vehicle enters a predefined test route  
2. Jetson evaluates the environment using camera + GNSS  
3. Remote operator receives live data  
4. Operator remotely enables AD‑ready mode  
5. Vehicle activates Level‑4/Level‑5 functionality  
6. Safety driver remains as fallback only  
7. Over time, Edge AI can automate steps 2–4  

---

## 🔭 Future Roadmap

- ROS2 integration  
- Multi‑camera support  
- Redundant perception modules  
- Automated ODD compliance checks  
- Full Edge‑AI‑based AD‑readiness decision engine  
- Remote operator dashboard (web‑based UI)  

---

## 📄 License
To be added based on your preference (MIT, Apache‑2.0, etc.)

---

## 🤝 Contributions
Contributions, discussions, and suggestions are welcome.  
Currently I single handle this project with help of GPT 4. Once project is matured, I definetly invite people to collaberate. 


----


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

## 🔍 Software Installation
### On Jetson Nano:

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

## 🌐 Future Enhancements
- Autonomous navigation with waypoints.
- Multi-camera support.
- Integration with Edge AI analytics.
