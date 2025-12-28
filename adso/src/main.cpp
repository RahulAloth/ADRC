#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "display_standalone_camera.h"
#include "camera_controller.h"
#include "camera_cntrl_mgr.h"
#include "detector_engine.h"

int main() {
    DisplayStandaloneCamera app;

    if (!app.init()) {
        return -1;
    }

    app.run();
    return 0;
}
