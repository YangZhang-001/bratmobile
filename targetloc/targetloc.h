#pragma once

#include "stereo.h"
#include "libcam2opencv.h"
#include "targetdet.h"
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera/camera_manager.h>
#include "c1lidarrpi.h"

class TargetLoc : public C1Lidar::DataInterface {
    public:

    void start();
    void stop();

    void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]);
    void onTargetDetected(std::vector<cv::Point2f> coord);

    Libcam2OpenCV cameraL;
    Libcam2OpenCV cameraR;

    libcamera::CameraManager cm;

    Stereo stereo;

    C1Lidar lidar;

    TargetDet targetDet;

    void updateImageL(const cv::Mat& l);

    void updateImageR(const cv::Mat& r);

    void updateStereo();

    cv::Mat currentL;
    cv::Mat currentR;
    cv::Mat currentD;

    const char* LIDAR_SERIAL_DEV = "/dev/ttyAMA0";
};
