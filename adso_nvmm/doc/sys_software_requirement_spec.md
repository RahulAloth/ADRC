# Multi‑Mode Jetson / DeepStream Pipeline Runner
### GPU‑Accelerated Video Processing & DeepStream Inference Pipeline Launcher

This document defines the system, functional, non‑functional, and pipeline‑level requirements for a GPU‑accelerated video processing application running on NVIDIA Jetson platforms.  
The application provides a multi‑mode GStreamer/DeepStream pipeline launcher supporting:

- NVMM zero‑copy GPU memory pipelines  
- Hardware‑accelerated decoding (NVDEC)  
- CUDA‑accelerated processing  
- DeepStream TensorRT inference  
- Headless inference for ROS2 integration  

The goal is to provide a modular, extensible, and production‑ready video pipeline suitable for edge‑AI systems.

---

# 1. System Overview

The system captures video from a V4L2 camera, processes it using NVIDIA GPU‑accelerated components, optionally performs TensorRT inference, and outputs either:

- GPU‑accelerated display (EGL)  
- Headless output for ROS2, logging, or backend processing  

## Supported Modes

| Mode | Purpose |
|------|---------|
| `--nvmm` | CPU JPEG decode → NVMM → GPU display |
| `--cuda_acc` | NVDEC hardware decode → NVMM → GPU display |
| `--cuda_process` | NVDEC → CUDA processing → GPU display |
| `--infer` | Full DeepStream inference pipeline with OSD |
| `--headless` | DeepStream inference without display (for ROS2) |

Each mode selects a different GStreamer pipeline optimized for specific use cases.

---

# 2. System Requirements (SyRS)

## 2.1 Hardware Requirements
- NVIDIA Jetson platform (Nano, Xavier NX, Orin NX, AGX Orin)
- Onboard GPU supporting:
  - NVDEC hardware decoding  
  - VIC hardware conversion  
  - CUDA compute  
- V4L2‑compatible camera (`/dev/video0`)
- Display (HDMI/DP) for non‑headless modes
- Minimum 4 GB RAM recommended

## 2.2 Operating System Requirements
- Ubuntu 20.04 / 22.04 (JetPack default)
- NVIDIA JetPack SDK (4.x or 5.x)
- DeepStream SDK (required for inference modes)

## 2.3 Software Dependencies
- GStreamer 1.16+
- NVIDIA GStreamer plugins:
  - `nvv4l2decoder`
  - `nvvidconv`
  - `nvvideoconvert`
  - `nvstreammux`
  - `nvinfer`
  - `nvdsosd`
  - `nveglglessink`
- GLib 2.0
- TensorRT (via DeepStream)
- CUDA runtime (for CUDA processing mode)

---

# 3. Functional Requirements (FR)

### FR‑1: Mode Selection
The system shall accept a command‑line argument specifying the pipeline mode.

### FR‑2: Pipeline Construction
The system shall construct a GStreamer pipeline string based on the selected mode and launch it using:

gst_parse_launch()

### FR‑3: Video Capture
The system shall capture video from:

v4l2src device=/dev/video0


### FR‑4: Hardware Decode Support
Depending on mode:

- CPU decode via `jpegdec` (NVMM mode)  
- NVDEC hardware decode via `nvv4l2decoder` (CUDA modes)

### FR‑5: GPU Memory Handling
All frames after decode shall be stored in:

video/x-raw(memory:NVMM)


### FR‑6: CUDA Processing
In `--cuda_process` mode:

nvvideoconvert cuda-process=true


### FR‑7: DeepStream Inference
In `--infer` and `--headless` modes:

- Use `nvstreammux` to batch frames  
- Run TensorRT inference via:

nvinfer config-file-path=<config>


- Support NV12 input format

### FR‑8: Display Output
Non‑headless modes shall render frames using:

nveglglessink


### FR‑9: Headless Output
Headless mode shall output to:

fakesink sync=false


### FR‑10: Error Handling
The system shall:

- Monitor the GStreamer bus  
- Handle EOS and ERROR messages  
- Print error/debug information  
- Terminate gracefully  

### FR‑11: Main Loop Management
The system shall:

- Create a GLib main loop  
- Run until EOS or ERROR  
- Clean up all GStreamer resources  

---

# 4. Non‑Functional Requirements (NFR)

### NFR‑1: Performance
- ≥ 30 FPS at 1280×720  
- Latency < 50 ms (non‑inference modes)  
- ≥ 20 FPS in inference mode  

### NFR‑2: Reliability
- No memory or GPU buffer leaks  
- Graceful recovery from camera disconnects (future extension)

### NFR‑3: Maintainability
- Modular CASE logic for pipeline selection  
- Pipelines easily extendable for new modes  

### NFR‑4: Portability
- Runs on any Jetson device with JetPack installed  
- No desktop‑only plugins  

### NFR‑5: Safety
- No blocking operations in the main loop  
- Avoid CPU‑GPU memory copies unless required  

### NFR‑6: Extensibility
Future enhancements include:

- ROS2 publishers  
- Multi‑camera support  
- Custom CUDA kernels  
- TensorRT engine hot‑swap  
- Logging and telemetry  

---

# 5. Pipeline Requirements (Per Mode)

## 5.1 Mode: `--nvmm`
- CPU JPEG decode  
- NVMM conversion  
- GPU display  
- No CUDA, no inference  

## 5.2 Mode: `--cuda_acc`
- NVDEC hardware decode  
- NVMM conversion  
- GPU display  

## 5.3 Mode: `--cuda_process`
- NVDEC hardware decode  
- CUDA‑accelerated processing  
- NVMM output  
- GPU display  

## 5.4 Mode: `--infer`
- NVDEC hardware decode  
- NVMM conversion  
- `nvstreammux` batching  
- TensorRT inference  
- OSD overlay  
- GPU display  

## 5.5 Mode: `--headless`
- Same as inference mode  
- Output to `fakesink`  
- No display  
- Suitable for ROS2 integration  

---

# 6. Future Requirements (Optional)

- **FR‑12:** ROS2 publisher for detection metadata  
- **FR‑13:** Multi‑camera muxing  
- **FR‑14:** Custom CUDA preprocessing kernels  
- **FR‑15:** TensorRT engine hot‑swap  
- **FR‑16:** Logging and telemetry system  

---

# 7. Acceptance Criteria

The system is acceptable when:

- All modes run without errors  
- NVMM memory flow is maintained  
- Inference mode produces bounding boxes  
- Headless mode runs without display  
- FPS meets performance requirements  
- No memory leaks occur  

---

# 8. High‑Level Data Flow

- Initialize GStreamer
- ↓
- Select pipeline mode (--nvmm / --cuda_acc / ...)
- ↓
- Build pipeline using gst_parse_launch()
- ↓
- Run inside GMainLoop
- ↓
- Handle EOS / ERROR via bus callback
- ↓
- Cleanup and shutdown


---

# 9. DeepStream Pipeline Explanation (Inference Modes)

## 1. Camera → JPEG
Your USB camera outputs JPEG frames:

    v4l2src device=/dev/video0


## 2. JPEG → NVDEC (Hardware Decoder)

    nvv4l2decoder mjpeg=1


- GPU hardware decode  
- Output: NV12  
- Stored in NVMM  

## 3. nvstreammux — DeepStream Batching

    nvstreammux batch-size=1 width=1280 height=720


Ensures:

- Batched tensor (batch=1)  
- Consistent resolution  
- TensorRT‑compatible memory layout  

## 4. nvvideoconvert — GPU Color Conversion

Converts NV12 → RGB, normalizes pixels, stays in NVMM.

## 5. nvinfer — TensorRT Inference

    nvinfer config-file-path=config_infer_primary.txt


Performs:

- Engine loading / building  
- Preprocessing  
- Inference  
- Metadata attachment  

## 6. nvdsosd — On‑Screen Display

Draws:

- Bounding boxes  
- Labels  
- Confidence scores  

## 7. nveglglessink — GPU Display

Zero‑copy GPU → display rendering.

---

# 10. End‑to‑End Deep Learning Flow

    Camera → JPEG → NVDEC → NV12 → RGB → TensorRT → Metadata → OSD → Display


Everything stays on the GPU:

- No CPU copies  
- No OpenCV  
- No Python overhead  
- No bottlenecks  

This is why DeepStream is widely used in robotics, ADAS, and edge‑AI deployments.

## DeepStream Templates
    Refer:
        - config_infer_primary_template.txt
        - config_infer_secondary.txt
    - This template works for TensorRT engines, ONNX models, or TAO .etlt models.
    - If we have to use TAO models (like TrafficCamNet), uncomment the TAO fields.
    - These templates are fully compatible with the current pipeline.
##  DeepStream Inference Pipeline

flowchart LR

    A[v4l2src<br/>Camera Input] --> B[image/jpeg Capsfilter]
    B --> C[nvv4l2decoder<br/>NVDEC Hardware Decode]
    C --> D[nvvidconv<br/>NV12 Conversion]
    D --> E[nvstreammux<br/>Batch=1, NVMM]
    E --> F[nvvideoconvert<br/>NV12 → RGB]
    F --> G[nvinfer<br/>TensorRT Inference]
    G --> H[nvdsosd<br/>On-Screen Display]
    H --> I[nveglglessink<br/>GPU Display]
