#include "display_standalone_camera.h"
#include <iostream>

DisplayStandaloneCamera::DisplayStandaloneCamera() {
    std::cout << "[ADSO] DisplayStandaloneCamera created." << std::endl;
}

bool DisplayStandaloneCamera::init() {
    std::cout << "[ADSO] System starting..." << std::endl;

    // ------------------------------------------------------------
    // 1. Create Camera Manager
    // ------------------------------------------------------------
    frontCam_ = std::make_shared<CameraController>("front_cam");

    // Set parameters BEFORE start()
    frontCam_->setParameter("width", "640");
    frontCam_->setParameter("height", "480");
    frontCam_->setParameter("fps", "30");

    camMgr_.addCameraController("front", frontCam_);

    std::cout << "[ADSO] Camera manager initialized." << std::endl;
    camMgr_.listCameraControllers();

    // Start camera
    if (!frontCam_->start(camIndex_)) {
        std::cerr << "[ADSO] ERROR: Could not start camera!" << std::endl;
        return false;
    }

    std::cout << "[ADSO] Camera started." << std::endl;

    // ------------------------------------------------------------
    // 2. Create Detector Engine (stub)
    // ------------------------------------------------------------
    detector_ = std::make_unique<DetectorEngine>("models/dummy.engine");

    if (!detector_->isLoaded()) {
        std::cerr << "[ERROR] Detector engine failed to load!" << std::endl;
        return false;
    }

    std::cout << "[ADSO] Detector engine initialized." << std::endl;

    return true;
}

void DisplayStandaloneCamera::run() {
    std::cout << "[ADSO] Entering main loop..." << std::endl;

    cv::Mat frame;

    while (true) {
        if (!frontCam_->captureImage(frame)) {
            std::cerr << "[ERROR] Empty frame received!" << std::endl;
            break;
        }

        // Run inference (stub)
        auto detections = detector_->infer(frame);

        // Draw detections (dummy)
        for (const auto& det : detections) {
            cv::rectangle(frame, det.bbox, cv::Scalar(0, 255, 0), 2);
            cv::putText(frame,
                        "ID:" + std::to_string(det.class_id),
                        det.bbox.tl(),
                        cv::FONT_HERSHEY_SIMPLEX,
                        0.6,
                        cv::Scalar(0, 255, 0),
                        2);
        }

        cv::imshow("ADSO Camera", frame);

        if (cv::waitKey(1) == 27)  // ESC
            break;
    }

    std::cout << "[ADSO] Shutting down..." << std::endl;
}
