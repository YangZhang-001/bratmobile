#pragma once

#include "c1lidarrpi.h"
#include "libcam2opencv.h"
#include "stereo.h"
#include "targetdet.h"
#include <libcamera/libcamera/camera_manager.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <vector>

class TargetLoc : public C1Lidar::DataInterface
{
  public:
    TargetLoc() = default;

    /**
     * Callback interface which reports a new target location.
     */
    struct DetectionInterface {
        /**
         * Callback that a new target has been detected.
         * @param topViewEgoCoordinage The top view egocentric xy coordinate in
         * meter.
         */
        virtual void
        newTargetDetected(const cv::Point2f topViewEgoCoordinate) = 0;
    };

    /**
     * Registers the detection callback.
     * @param cb Pointer to the callback interface.
     */
    void registerNewTargetDetectedCallback(DetectionInterface *cb)
    {
        detectionInterface = cb;
    }

    void start();
    void stop();

    /**
     * Needs to be registered with the LIDAR from main().
     * This is the LIDAR callback provding this class with fresh
     * LIDAR data!
     */
    void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]);

    /**
     * For debugging purposes and the GUI we can get the current left camera
     * image. This is pure polling but thread-safe.
     */
    const cv::Mat getCurrentLCameraImage()
    {
        std::lock_guard<std::mutex> guard(leftImage_mutex);
        return currentL;
    }

    /**
     * For debugging purposes and the GUI we can get the current right camera
     * image. This is pure polling but thread-safe.
     */
    const cv::Mat getCurrentRCameraImage()
    {
        std::lock_guard<std::mutex> guard(rightImage_mutex);
        return currentR;
    }

    /**
     * For debugging purposes and the GUI we can get the current disparity
     * readings. This is pure polling but thread-safe.
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

    const std::vector<cv::Point> getQRcodeContour()
    {
        std::lock_guard<std::mutex> guard(contour_mutex);
        return scaledContour;
    }

    static constexpr int LIDAR_DATA_POINTS = C1Lidar::nDistance;

    /**
     * How inverse disparity maps to meter.
     **/
    const float disp2meter = 500;

    /**
     * Factor which maps the x-pixel pos to meter
     **/
    const float xpos2meter = 1.0 / 2000.0;

    /**
     * The x pixel position in the camera image where it's the centre pos
     */
    const float xposAtCentre = 1140;

  private:
    cv::Mat currentL;
    cv::Mat currentR;
    cv::Mat currentD;

    const char *LIDAR_SERIAL_DEV = "/dev/ttyAMA0";

    void onTargetDetected(const std::vector<cv::Point2f> &coords);

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

    std::deque<std::vector<cv::Point2f>> contoursRingbuffer;

    DetectionInterface *detectionInterface = nullptr;

    float point2point(cv::Point2f a, cv::Point2f b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return sqrt(dx * dx + dy * dy);
    }

    // error in pixels between contours
    const float maxContourPixelErrorBetweenDetectionContours = 10;
};
