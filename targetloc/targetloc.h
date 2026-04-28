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
    TargetLoc () = default;

    /**
     * Callback interface which reports a new target location.
     */
    struct DetectionInterface
    {
        /**
         * Callback that a new target has been detected.
         * @param topViewEgoCoordinage The top view xy coordinate in meter.
         */
        virtual void newTargetDetected (const cv::Point2f topViewEgoCoordinate)
            = 0;
    };

    /**
     * Registers the detection callback.
     * @param cb Pointer to the callback interface.
     */
    void
    registerNewTargetDetectedCallback (DetectionInterface *cb)
    {
        detectionInterface = cb;
    }

    /**
     * Starts the cameras and the lidar.
     */
    void start ();

    /**
     * Stops the cameras and the lidar.
     */
    void stop ();

    /**
     * Needs to be registered with the LIDAR from main().
     * This is the LIDAR callback provding this class with fresh
     * LIDAR data!
     */
    void newScanAvail (C1LidarData (&data)[C1Lidar::nDistance]);

    /**
     * For debugging purposes we can get the current left camera
     * image.
     */
    const cv::Mat getCurrentLCameraImage ()
    {
        std::lock_guard<std::mutex> guard (leftImage_mutex);
        return currentL;
    }

    /**
     * For debugging purposes we can get the current right camera
     * image.
     */
    const cv::Mat getCurrentRCameraImage ()
    {
        std::lock_guard<std::mutex> guard (rightImage_mutex);
        return currentR;
    }

    /**
     * For debugging purposes we can get the current disparity
     * readings.
     */
    const cv::Mat getCurrentDisparityMap ()
    {
        std::lock_guard<std::mutex> guard (disparityData_mutex);
        return currentD;
    }

    /**
     * Gets all current valid LIDAR coordinates.
     */
    const std::vector<cv::Point2f> getCurrentLidarCoords ()
    {
        std::lock_guard<std::mutex> guard (lidarData_mutex);
        return currentLidarCoords;
    }

    /**
     * Gets the countour around the detected target.
     */
    const std::vector<cv::Point> getQRcodeContour ()
    {
        std::lock_guard<std::mutex> guard (contour_mutex);
        return scaledContour;
    }

    /**
     * Mirrored from the LIDAR driver: the max number of points
     */
    static constexpr int LIDAR_DATA_POINTS = C1Lidar::nDistance;

    /**
     * How inverse disparity maps to meter.
     **/
    const float disp2meter = 500;

    /**
     * Factor which maps the x-pixel camerea image pos to meter.
     **/
    const float xpos2meter = 1.0 / 2000.0;

    /**
     * The x pixel position of the camera image where it's the centre pos.
     */
    const float xposAtCentre = 1140;

  private:
    // current left camera image
    cv::Mat currentL;
    
    // current right camera image
    cv::Mat currentR;

    // current disparity map
    cv::Mat currentD;

    // Serial device of the LIDAR
    const char *LIDAR_SERIAL_DEV = "/dev/ttyAMA0";

    // Callback when a target has been detected
    void onTargetDetected (const std::vector<cv::Point2f> &coords);

    // Left camera
    Libcam2OpenCV cameraL;

    // Right camera
    Libcam2OpenCV cameraR;

    // The common settings for both cameras
    Libcam2OpenCVSettings settings;

    // Cameramanager for both cameras
    libcamera::CameraManager cm;

    // Stereo detector
    Stereo stereo;

    // Target detector
    TargetDet targetDet;

    // current LIDAR coordinates updated by the LIDAR callback
    std::vector<cv::Point2f> currentLidarCoords;

    // callback from libcamera with a fresh left image
    void updateImageL (const cv::Mat &l);

    // callback from libcamera with a fresh right image
    void updateImageR (const cv::Mat &r);

    // Triggers the calculation of the disparity.
    void updateStereo ();

    // Mutexes for the getters so that the
    // data exists while getting it.
    std::mutex lidarData_mutex;
    std::mutex disparityData_mutex;
    std::mutex leftImage_mutex;
    std::mutex rightImage_mutex;
    std::mutex contour_mutex;

    // Current contour around the target in the resolution
    // of the disparity map.
    std::vector<cv::Point> scaledContour;

    // ringbuffer of countours to check if they are consistently det
    std::deque<std::vector<cv::Point2f> > contoursRingbuffer;

    // Detection Callback
    DetectionInterface *detectionInterface = nullptr;

    // Cartesian distance between two points
    float point2point (cv::Point2f a, cv::Point2f b)
    {
        const float dx = a.x - b.x;
        const float dy = a.y - b.y;
        return sqrt (dx * dx + dy * dy);
    }

    // Max error in pixels between consecutive contours
    const float maxContourPixelErrorBetweenDetectionContours = 10;
};
