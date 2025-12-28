#pragma once

#include <string>
#include <vector>
#include <memory>

#include <opencv4/opencv2/imgproc.hpp> // cv::resize
#include <opencv4/opencv2/opencv.hpp> // umbrella
#include <opencv4/opencv2/core.hpp> // cv::Mat, cv::Rect

// #include <NvInfer.h> // TensorRT main header (commented out as placeholder)


// ------------------------------------------------------------
// Detection struct
// Represents a single detection result from the model
// ------------------------------------------------------------
struct Detection {
    int class_id = -1;          // Class index
    float confidence = 0.0f;    // Confidence score
    
    cv::Rect bbox;              // Bounding box in pixel coordinates
};

// ------------------------------------------------------------
// DetectorEngine
// A generic inference engine wrapper for TensorRT models
// ------------------------------------------------------------
class DetectorEngine {
public:
    // Constructor: takes path to TensorRT engine file
    explicit DetectorEngine(const std::string& engine_path);

    // Destructor
    ~DetectorEngine();

    // Load TensorRT engine from file
    bool loadEngine(const std::string& path);

    // Check if engine is loaded
    bool isLoaded() const;

    // Run inference on a frame and return detections
    std::vector<Detection> infer(const cv::Mat& frame);

private:
    // Preprocess input frame (resize, normalize, etc.)
    cv::Mat preprocess(const cv::Mat& frame);

    // Postprocess raw model output into Detection objects
    std::vector<Detection> postprocess();

private:
    std::string engine_path_;   // Path to TensorRT engine file

    // Placeholder for TensorRT objects (runtime, engine, context)
    // std::unique_ptr<nvinfer1::IRuntime> runtime_;
    // std::unique_ptr<nvinfer1::ICudaEngine> engine_;
    // std::unique_ptr<nvinfer1::IExecutionContext> context_;

    // Model input dimensions (set these based on your model)
    int input_width_  = 640;
    int input_height_ = 640;
};
