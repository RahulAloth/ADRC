# Stereo Vision Pipeline — GStreamer → OpenCV → VPI (CUDA)
Author: Rahul  
Platform: Jetson (JetPack 6.1, VPI 3.x)  
Cameras: Dual USB (YUY2)  
Output: CUDA Stereo Disparity (Q10.5 → float)

---

## 1. Overview

This document describes a complete stereo vision pipeline built on NVIDIA Jetson using:

- **GStreamer** for camera capture  
- **OpenCV** for YUY2 → grayscale conversion  
- **VPI (CUDA backend)** for stereo disparity  
- **Q10.5 disparity output** converted to float  
- **Two independent pipelines** for left and right cameras  
- **Right camera triggers disparity computation**  

The system is designed for USB cameras that output **YUY2** in **system memory**.

---

## 2. High-Level Architecture
```cpp
Left Camera (/dev/video0)          Right Camera (/dev/video2)
│                                   │
▼                                   ▼
GStreamer Pipeline                  GStreamer Pipeline
│                                   │
▼                                   ▼
appsink (left)                      appsink (right)
│                                   │
▼                                   ▼
sample_to_gray()                   sample_to_gray()
│                                   │
▼                                   ▼
ctx.left  (cv::Mat)                 ctx.right  (cv::Mat)
│                                   │
└─────────────── disparity_thread() ────────────────►
│
▼
VPI CUDA Stereo Disparity
│
▼
disparity.png
```

---

## 3. GStreamer Pipelines

Each camera has its own pipeline:

v4l2src → nvvidconv → tee → (display)
→ (appsink)


### Left Pipeline
- Device: `/dev/video0`
- appsink callback: `on_new_sample_left`

### Right Pipeline
- Device: `/dev/video2`
- appsink callback: `on_new_sample_right`
- Triggers disparity computation

---

## 4. sample_to_gray(): Converting GstSample → cv::Mat

This function:

1. Maps the GStreamer buffer  
2. Reads width/height from caps  
3. Detects format (YUY2 or GRAY8)  
4. Converts YUY2 → grayscale using OpenCV  
5. Returns a clean `cv::Mat gray`

### Key detail: **Correct stride handling**

USB cameras often pad rows.  
Correct stride is:

stride = vmeta->stride[0]


If no metadata:

stride = width * 2


---

## 5. Left Camera Callback

```cpp

on_new_sample_left()
Converts sample to grayscale

Stores into ctx.left

Stores timestamp

Does not run disparity

```

## 6. Right Camera Callback

on_new_sample_right()

    Converts sample to grayscale

    Stores into ctx.right

    Calls disparity_thread(&ctx)

Right camera drives the stereo pipeline.
## 7. disparity_thread(): Core Stereo Logic
This function performs:
- 7.1 Frame readiness check
- Ensures both left and right frames exist.
- 7.2 Wrap OpenCV Mats into VPI images
- vpiImageCreateWrapperOpenCVMat(ctx->left,  VPI_IMAGE_FORMAT_U8, 0, &leftVPI);
- vpiImageCreateWrapperOpenCVMat(ctx->right, VPI_IMAGE_FORMAT_U8, 0, &rightVPI);

## 8. VPI Stream
- A VPI stream is a command queue for CUDA/CPU operations.
- vpiStreamCreate(0, &stream);
## 9. Important Notes
    - vpiImageCreateWrapperOpenCVMat works with CUDA
    - USB cameras produce padded YUY2 → stride must be handled
    - Right camera triggers disparity
    - Disparity output is Q10.5 fixed‑point
    - Convert using: float = int16 / 32.0
## 10. TODO

    - Add confidence map
    - Add left/right timestamp synchronization
    - Switch to NVMM for zero‑copy GPU processing
    - Add colorized disparity visualization
    - Add rectification (if cameras are not hardware-aligned)

## 11. Summary
This pipeline is a complete, production-grade stereo vision system using:
- Dual USB cameras
- GStreamer
- OpenCV
- VPI (CUDA backend)
- Q10.5 disparity output
It is optimized for JetPack 6.1 and VPI 3.x, and uses only supported APIs.
