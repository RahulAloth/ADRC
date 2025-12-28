# 🚗 Jetson Orin ROS2 Perception Pipeline  
### Camera → Object Detection → Traffic Detection → People Detection → Depth Analysis  
### Architecture: Modular ROS2 Nodes • Zero‑Copy (NITROS) • Composable Nodes • Graph‑Based Pipeline

---

## 📌 1. System Overview

This project implements a **ROS2‑based perception pipeline** on **Jetson Orin (8GB, JetPack 6.0)** using:

- **USB Camera** for RGB input  
- **TensorRT** for high‑performance inference  
- **NITROS** for zero‑copy GPU data transport  
- **Composable ROS2 nodes** for low‑latency in‑process execution  
- **Graph‑based (DAG) architecture** for clarity and scalability  

The pipeline performs:

- Object Detection  
- Traffic Object Detection (vehicles, signs, lights)  
- People Detection  
- Depth Estimation (monocular or stereo in future)

---

## 📐 2. High‑Level Dataflow (DAG)

```text
USB Camera
   ↓
[Camera Node] → /camera/image_raw
   ↓
[Preprocess Node] → /vision/image_tensor
   ↓
[Detection Node (TensorRT)] → /vision/detections
   ↓
[Semantic Split Node]
      ├── /vision/traffic_detections
      └── /vision/people_detections

Parallel Branch:
[Depth Node] → /vision/depth_map


```

## 📊 Perception Pipeline Diagram (Mermaid)

```mermaid
flowchart TD

    %% Camera Input
    A[USB Camera] --> B[Camera Node\nPublishes: /camera/image_raw]

    %% Preprocessing
    B --> C[Preprocess Node\nPublishes: /vision/image_tensor]

    %% Detection
    C --> D[Detection Node - TensorRT\nPublishes: /vision/detections]

    %% Semantic Split
    D --> E1[Traffic Detection Output\nPublishes: /vision/traffic_detections]
    D --> E2[People Detection Output\nPublishes: /vision/people_detections]

    %% Depth Branch (Parallel)
    B --> F[Depth Node\nPublishes: /vision/depth_map]

    %% Optional downstream nodes
    E1 --> G1[Traffic Tracking Node]
    E2 --> G2[People Tracking Node]
    F --> H[Depth Fusion Node\nPublishes: /vision/depth_at_detections]

    %% Visualizer
    G1 --> V[Visualizer Node\nRViz2 or Foxglove]
    G2 --> V
    H --> V


