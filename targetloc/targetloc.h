#pragma once

#include "stereo.h"
#include "libcam2opencv.h"
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera/camera_manager.h>
#include "c1lidarrpi.h"

const char RPI_SERIAL_DEV[] = "/dev/ttyAMA0";

class TargetLoc : public C1Lidar::DataInterface {
    public:
    
    void start();
    void stop();

    void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]);

    private:

    Libcam2OpenCV cameraL;
    Libcam2OpenCV cameraR;

    libcamera::CameraManager cm;

    Stereo stereo;

    C1Lidar lidar;
};
