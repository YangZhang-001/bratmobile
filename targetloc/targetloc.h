#pragma once

#include "stereo.h"
#include "libcam2opencv.h"
#include "targetdet.h"
#include <opencv2/opencv.hpp>
#include <libcamera/libcamera/camera_manager.h>
#include "c1lidarrpi.h"
#include <mutex>
#include <vector>
#include <functional>

class TargetLoc : public C1Lidar::DataInterface
{
public:
    TargetLoc() = default;

    struct DetectionInterface
    {
        virtual void newTargetDetected(const float x, const float y) = 0;
    };

    void registerNewTargetDetectedCallback(DetectionInterface* cb) {
	detectionInterface = cb;
    }

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
    const cv::Mat getCurrentDisparityMap()
    {
	std::lock_guard<std::mutex> guard(disparityData_mutex);
        return currentD;
    }

    const std::vector<cv::Point2f> getCurrentLidarCoords()
    {
        std::lock_guard<std::mutex> guard(lidarData_mutex);
        return currentLidarCoords;
    }

    const std::vector<cv::Point> getQRcodeContour() {
        std::lock_guard<std::mutex> guard(contour_mutex);
	return scaledContour;
    }

    static constexpr int LIDAR_DATA_POINTS = C1Lidar::nDistance;

    /**
     * How inverse disparity maps to meter.
     **/
    float disp2meter = 500;

    /**
     * Factor which maps the x-pixel pos to meter
     **/
    float xpos2meter = 1.0/400.0;

private:
    cv::Mat currentL;
    cv::Mat currentR;
    cv::Mat currentD;

    const char *LIDAR_SERIAL_DEV = "/dev/ttyAMA0";

    void onTargetDetected(const std::vector<cv::Point2f>& coords);

    Libcam2OpenCV cameraL;
    Libcam2OpenCV cameraR;
    Libcam2OpenCVSettings settings;

    libcamera::CameraManager cm;

    Stereo stereo;

    TargetDet targetDet;

    std::vector<cv::Point2f> currentLidarCoords;

    void updateImageL(const cv::Mat &l);

    void updateImageR(const cv::Mat &r);

    void updateStereo();

    std::mutex lidarData_mutex;
    std::mutex disparityData_mutex;
    std::mutex leftImage_mutex;
    std::mutex rightImage_mutex;
    std::mutex contour_mutex;

    std::vector<cv::Point> scaledContour;

    DetectionInterface* detectionInterface = nullptr;
};
