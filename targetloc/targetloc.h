#pragma once

#include "stereo.h"
#include "libcam2opencv.h"
#include "targetdet.h"
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera/camera_manager.h>
#include "c1lidarrpi.h"

class TargetLoc : public C1Lidar::DataInterface
{
public:
    TargetLoc() = default;

    struct DetectionInterface
    {
        virtual void hasNewTargetDetection(float x, float y) = 0;
    };

    void start();
    void stop();

    /**
     * Needs to be registered with the LIDAR from main()
     */
    void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]);

    /**
     * For debugging purposes and the GUI we can get the current left camera image
     */
    const cv::Mat getCurrentLCameraImage() const
    {
        return currentL;
    }

    /**
     * For debugging purposes and the GUI we can get the current right camera image
     */
    const cv::Mat getCurrentRCameraImage() const
    {
        return currentR;
    }

    /**
     * For debugging purposes and the GUI we can get the current disparity readings
     */
    const cv::Mat getCurrentDisparityMap() const
    {
        return currentD;
    }

    const std::vector<cv::Point2f> getCurrentLidarCoords() const
    {
        return currentLidarCoords;
    }

    private:
    cv::Mat currentL;
    cv::Mat currentR;
    cv::Mat currentD;

    const char *LIDAR_SERIAL_DEV = "/dev/ttyAMA0";

    void onTargetDetected(const cv::Point2f coord);

    Libcam2OpenCV cameraL;
    Libcam2OpenCV cameraR;

    libcamera::CameraManager cm;

    Stereo stereo;

    TargetDet targetDet;

    std::vector<cv::Point2f> currentLidarCoords;

    void updateImageL(const cv::Mat &l);

    void updateImageR(const cv::Mat &r);

    void updateStereo();
};
