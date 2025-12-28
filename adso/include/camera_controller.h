#pragma once

#include <string>          // <-- REQUIRED for std::string
#include <map>             // <-- REQUIRED for std::map
#include <memory>          // <-- REQUIRED for std::shared_ptr
#include <mutex>

#include <opencv4/opencv2/opencv.hpp>

class CameraController {
public:
    explicit CameraController(const std::string& name);
    ~CameraController();

    std::string getName() const;
    void setName(const std::string& name);

    bool start(int device_id = 0);      // Start camera
    void stop();                        // Stop camera
    bool captureImage(cv::Mat& out);    // Capture frame

    void setParameter(const std::string& param, const std::string& value);
    std::string getParameter(const std::string& param) const;
    bool display(const std::string& window_name, int delay_ms = 1);


private:
    std::string name_;
    std::map<std::string, std::string> parameters_;
    cv::VideoCapture cap_;
    bool is_running_ = false;
    mutable std::mutex mtx_; // <-- THREAD SAFETY
};
