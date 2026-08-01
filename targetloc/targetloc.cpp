#include "targetloc.h"
#include <chrono> // rock5 stereo throttling timing
#include <opencv2/core/utility.hpp> // limit and report opencv thread count
#include <opencv2/core/types.hpp>
#include <mutex>  //protect stereo throttle and startup

#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
#include "rock5_V4Lcamera_backend.h"
#include <linux/videodev2.h>
#include <opencv2/videoio.hpp>
#else
#include <libcamera/libcamera/camera_manager.h>
#endif

#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
namespace {
// limit opencv threads on rock5, 4 threads for opencv
// the default opencv thread count is 8, which caused high load 
// when stereo, camera capture, lidar and QT were running together
constexpr int ROCK5_OPENCV_THREADS  = 4;

// limit rock5 stereo disparity calculation to 1 per second
// reduce the peak and accumulated load from StereoSGBM, on rock5
constexpr int ROCK5_STEREO_INTERVAL_SECONDS = 1;

}
#endif

void TargetLoc::start()
{
#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
    // limit opencv threads on rock5, 4 threads for opencv
    cv::setNumThreads(ROCK5_OPENCV_THREADS);

    // print the active rock5 load-control settings. used for testing
    printf("Rock5 load control: opencv_threads=%d, stereo_interval_s=%d, cpu_count=%d\n", 
        cv::getNumThreads(), ROCK5_STEREO_INTERVAL_SECONDS, cv::getNumberOfCPUs());
    fflush(stdout);
#endif

    targetDet.registerDetCallback([&](const std::vector<cv::Point2f> &coords) {
        onTargetDetected(coords);
    });

    stereo.registerCallback([&](cv::Mat d) {
        std::lock_guard<std::mutex> guard(disparityData_mutex);
        currentD = d;
    });

// start cameras in rock5 model
#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA

    // regester frame callbacks for left/right camera
    cameraL.registerFrameCallback([&](const cv::Mat &left) {
            updateImageL(left);
        });
    cameraR.registerFrameCallback([&](const cv::Mat &right) {
            updateImageR(right);
        });

    V4L2OpenCVParameters leftParameters;
    leftParameters.width = 1920;
    leftParameters.height = 1080;
    leftParameters.deviceID = 23;
    leftParameters.fourcc = cv::VideoWriter::fourcc('N', 'V', '1', '2');
    leftParameters.framerate = 10;

    V4L2OpenCVParameters rightParameters = leftParameters;
    rightParameters.deviceID = 32;

    const std::vector<V4L2ControlParameter> leftControls = {
        // Keep exposure and gain moderate to reduce noise and overbright images
        // exposure: 0~4095, gain: 256~43663.
        {"/dev/v4l-subdev2", V4L2_CID_EXPOSURE, 0.25},
        {"/dev/v4l-subdev2", V4L2_CID_GAIN, 0.12},
        {"/dev/v4l-subdev2", V4L2_CID_HFLIP, 1},
        {"/dev/v4l-subdev2", V4L2_CID_VFLIP, 1}
    };

    const std::vector<V4L2ControlParameter> rightControls = {
        {"/dev/v4l-subdev7", V4L2_CID_EXPOSURE, 0.25},
        {"/dev/v4l-subdev7", V4L2_CID_GAIN, 0.12},
        {"/dev/v4l-subdev7", V4L2_CID_HFLIP, 1},
        {"/dev/v4l-subdev7", V4L2_CID_VFLIP, 1}
    };
    // start left camera
    const V4L2OpenCVParameters actualLeft = cameraL.start(leftParameters, leftControls);

    // start right camera
    const V4L2OpenCVParameters actualRight = cameraR.start(rightParameters, rightControls);

    const int leftWidth =
        actualLeft.width > 0 ? actualLeft.width : leftParameters.width;
    const int leftHeight =
        actualLeft.height > 0 ? actualLeft.height : leftParameters.height;
    const int rightWidth =
        actualRight.width > 0 ? actualRight.width : rightParameters.width;
    const int rightHeight =
        actualRight.height > 0 ? actualRight.height : rightParameters.height;

    cameraWidth = leftWidth;
    cameraHeight = leftHeight;

    if ((leftWidth != rightWidth) || (leftHeight != rightHeight)) {
        fprintf(stderr, "Warning: Left and right camera have different resolution! Left: %dx%d, Right: %dx%d\n",
                leftWidth, leftHeight, rightWidth, rightHeight);
    }


#else
    cm.start();

    cameraL.registerCallback(
        [&](const cv::Mat &left, const libcamera::ControlList &) {
            updateImageL(left);
        });

    cameraR.registerCallback(
        [&](const cv::Mat &right, const libcamera::ControlList &) {
            updateImageR(right);
        });



    settings.width = 1920;
    settings.height = 1080;
    cameraWidth = settings.width;
    cameraHeight = settings.height;

    settings.cameraIndex = 0;
    cameraL.start(cm, settings);

    settings.cameraIndex = 1;
    cameraR.start(cm, settings);
#endif
}

void TargetLoc::stop()
{
    cameraL.stop();
    cameraR.stop();

#ifndef TARGETLOC_USE_ROCK5_V4L_CAMERA
    // stop cameras in Raspiberry Pi model
    cm.stop();
#endif

}

void TargetLoc::newScanAvail(C1LidarData (&data)[C1Lidar::nDistance])
{
    std::lock_guard<std::mutex> guard(lidarData_mutex);
    currentLidarCoords.clear();
    for (const auto &v : data) {
        if (v.valid) {
            currentLidarCoords.push_back({v.x, v.y});
        }
    }
}

void TargetLoc::updateStereo()
{
    // let's check if we have really images from both cameras!
    if (currentL.empty())
        return;
    if (currentR.empty())
        return;
    if (currentL.size != currentR.size)
        return;

#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
    // camera L&R callbcks may both call updatestereo()
    static std::mutex stereoStartMutex;
     
    // On Rock 5, avoid starting StereoSGBM too frequently.
    // This keeps the load close to the tested 1Hz configuration.
    static auto lastStereoTime =
        std::chrono::steady_clock::now() -
        std::chrono::seconds(ROCK5_STEREO_INTERVAL_SECONDS);

    std::lock_guard<std::mutex> stereoStartLock(stereoStartMutex);

    const auto now = std::chrono::steady_clock::now();

    if (now - lastStereoTime <
        std::chrono::seconds(ROCK5_STEREO_INTERVAL_SECONDS))
        return;

    lastStereoTime = now;
#endif

    // yes, we have!
    stereo.calcDepthMapAsync(currentL, currentR);
}

void TargetLoc::updateImageL(const cv::Mat &l)
{
    leftImage_mutex.lock();
    currentL = l;
    leftImage_mutex.unlock();
    updateStereo();
    // We detect the target from the left eye!
    targetDet.detectAsync(l);
}

void TargetLoc::updateImageR(const cv::Mat &r)
{
    rightImage_mutex.lock();
    currentR = r;
    rightImage_mutex.unlock();
    updateStereo();
}

// here it's where it's getting interesting!
void TargetLoc::onTargetDetected(const std::vector<cv::Point2f> &contour)
{
    // removing scaled contour which is used here for debuggin and visualisation
    // doing it in a thread-safe way in case it's being plotted by the QT GUI.
    contour_mutex.lock();
    scaledContour.clear();
    contour_mutex.unlock();

    // We put the detection contours into a double ended queue of 3
    // and check if all the detection points are within an error margin
    // of maxContourPixelErrorBetweenDetectionContours.
    contoursRingbuffer.push_front(contour);
    if (contoursRingbuffer.size() < 3) {
        return;
    }
    contoursRingbuffer.pop_back();
    for (int i = 1; i < 3; i++) {
        std::vector<cv::Point2f> contour1 = contoursRingbuffer[i - 1];
        std::vector<cv::Point2f> contour2 = contoursRingbuffer[i];
        for (unsigned long int j = 0;
             (j < contour1.size()) && (j < contour2.size()); j++) {
            if (point2point(contour1[j], contour2[j]) >
                maxContourPixelErrorBetweenDetectionContours) {
                fprintf(stderr, "Contour discarded.\n");
                return;
            }
        }
    }

    // Checking if the disparity map is actually there as we need it to find out
    // how far the target is.
    if (currentD.empty())
        return;

    // We need to scale the contour from full resolution to the resolution
    // of the disparity map.
    printf("We have a contour around a target:");
    int i = 0;
    float avgX = 0;
    contour_mutex.lock();
    for (auto &c : contour) {
        const int x = c.x * currentD.size().width / cameraWidth;
        const int y = c.y * currentD.size().height / cameraHeight;
        scaledContour.emplace_back(x, y);
        printf("[%d,%d]", x, y);
        avgX = avgX + c.x;
        i++;
    }
    contour_mutex.unlock();
    avgX = avgX / i;
    printf(", avgX = %f",avgX);
    printf("\n");

    // Create mask
    cv::Mat mask = cv::Mat::zeros(currentD.size(), CV_8UC1);

    // Draw filled contour for the mask
    std::vector<std::vector<cv::Point>> scaledContours{scaledContour};
    cv::drawContours(mask, scaledContours, -1, cv::Scalar(255), cv::FILLED);

    // Compute mean disparity value inside contour
    float avgDisp = cv::mean(currentD, mask)[0];

    cv::Point2f targetLoc;

    // mapping disparity to distance in meter for the LIDAR / egocentric
    // x-coordinate
    targetLoc.x = disp2meter / avgDisp;

    // mapping the x-coordinate pixels of the camera to the LIDAR y-coordinate
    // of the target in meter
    targetLoc.y = xpos2meter * (xposAtCentre - avgX);

    printf("Disparity: %f, Target location: [%f,%f]\n", avgDisp, targetLoc.x,
           targetLoc.y);

    // callback!
    if (detectionInterface) {
        detectionInterface->newTargetDetected(targetLoc);
    }
}
