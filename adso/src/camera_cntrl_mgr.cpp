#include "camera_cntrl_mgr.h"
#include <memory>
#include <vector>
#include <algorithm>
#include <iostream>

CameraCntrlMgr::CameraCntrlMgr() = default;
CameraCntrlMgr::~CameraCntrlMgr() = default;

void CameraCntrlMgr::addCameraController(const std::string& id, std::shared_ptr<CameraController> controller) {
    controllers_[id] = controller;
}

void CameraCntrlMgr::removeCameraController(const std::string& id) {
    controllers_.erase(id);
}

std::shared_ptr<CameraController> CameraCntrlMgr::getCameraController(const std::string& id) const {
    auto it = controllers_.find(id);
    if (it != controllers_.end()) {
        return it->second;          
    }
    return nullptr;
}


void CameraCntrlMgr::listCameraControllers() const {
    std::cout << "Camera Controllers:" << std::endl;
    for (const auto& pair : controllers_) {
        std::cout << "ID: " << pair.first << std::endl;
    }
    
}