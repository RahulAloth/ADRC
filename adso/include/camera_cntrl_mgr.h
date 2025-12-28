#pragma once

#include <string>
#include <map>
#include <memory>

#include "camera_controller.h"   // for CameraController

class CameraCntrlMgr {
public:
    CameraCntrlMgr();
    ~CameraCntrlMgr();

    void addCameraController(const std::string& id, std::shared_ptr<CameraController> controller);
    void removeCameraController(const std::string& id);
    std::shared_ptr<CameraController> getCameraController(const std::string& id) const;
    void listCameraControllers() const;

private:
    std::map<std::string, std::shared_ptr<CameraController>> controllers_;
};
