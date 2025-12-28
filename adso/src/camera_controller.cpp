#include "camera_controller.h"
#include <iostream>

CameraController::CameraController(const std::string& name)
    : name_(name) {}

CameraController::~CameraController() {
    stop();
}

std::string CameraController::getName() const {
    return name_;
}

void CameraController::setName(const std::string& name) {
    name_ = name;
}

bool CameraController::start(int device_id) {
    std::lock_guard<std::mutex> lock(mtx_);    
    if (is_running_) {
        std::cout << "[CameraController] Camera already running." << std::endl;
        return true;
    }

    std::cout << "[CameraController] Opening camera: " << device_id << std::endl;

    // Open USB camera
    cap_.open(device_id);

    if (!cap_.isOpened()) {
        std::cerr << "[CameraController] ERROR: Could not open camera!" << std::endl;
        return false;
    }

    // Optional: apply parameters
    if (parameters_.count("width"))
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, std::stoi(parameters_["width"]));

    if (parameters_.count("height"))
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, std::stoi(parameters_["height"]));

    if (parameters_.count("fps"))
        cap_.set(cv::CAP_PROP_FPS, std::stoi(parameters_["fps"]));

    is_running_ = true;
    std::cout << "[CameraController] Camera started successfully." << std::endl;

    return true;
}

void CameraController::stop() {
    std::lock_guard<std::mutex> lock(mtx_);
    if (is_running_) {
        cap_.release();
        is_running_ = false;
        std::cout << "[CameraController] Camera stopped." << std::endl;
    }
}

bool CameraController::captureImage(cv::Mat& out) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!is_running_) {
        std::cerr << "[CameraController] ERROR: Camera not started!" << std::endl;
        return false;
    }

    cap_ >> out;

    if (out.empty()) {
        std::cerr << "[CameraController] ERROR: Empty frame captured!" << std::endl;
        return false;
    }

    return true;
}
void CameraController::setParameter(const std::string& param, const std::string& value) {
    std::lock_guard<std::mutex> lock(mtx_);
    parameters_[param] = value;
}
std::string CameraController::getParameter(const std::string& param) const {
    std::lock_guard<std::mutex> lock(mtx_);
    auto it = parameters_.find(param);
    if (it != parameters_.end()) {
        return it->second;
    }
    return "";
}

bool CameraController::display(const std::string& window_name, int delay_ms) {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!is_running_) {
        std::cerr << "[CameraController] ERROR: Camera not started!" << std::endl;
        return false;
    }

    cv::Mat frame;
    if (!captureImage(frame)) {
        return false;
    }

    cv::imshow(window_name, frame);
    cv::waitKey(delay_ms);
    return true;
}
