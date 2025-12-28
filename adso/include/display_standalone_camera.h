#pragma once

#include <memory>
#include <opencv2/opencv.hpp>

#include "camera_controller.h"
#include "camera_cntrl_mgr.h"
#include "detector_engine.h"

class DisplayStandaloneCamera {
public:
    DisplayStandaloneCamera();
    ~DisplayStandaloneCamera() = default;

    bool init();
    void run();

private:
    CameraCntrlMgr camMgr_;
    std::shared_ptr<CameraController> frontCam_;
    std::unique_ptr<DetectorEngine> detector_;

    int camIndex_ = 0;  // default USB camera index
};
