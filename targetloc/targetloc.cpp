#include "targetloc.h"
#include <chrono> // rock5 stereo throttling timing
#include <opencv2/core/utility.hpp> // limit and report opencv thread count
#include <opencv2/core/types.hpp>
#include <mutex>  //protect stereo throttle and startup
#include <algorithm> // std::sort
#include <cmath> // std::fabs, std::hypot, std::atan2

#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
#include "rock5_V4Lcamera_backend.h"
#include <linux/videodev2.h>
#include <opencv2/videoio.hpp>
#else
#include <libcamera/libcamera/camera_manager.h>
#endif

namespace {
#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
// limit opencv threads on rock5, 4 threads for opencv
// the default opencv thread count is 8, which caused high load 
// when stereo, camera capture, lidar and QT were running together
constexpr int ROCK5_OPENCV_THREADS  = 4;

// limit rock5 stereo disparity calculation to 1 per second
// reduce the peak and accumulated load from StereoSGBM, on rock5
constexpr int ROCK5_STEREO_INTERVAL_SECONDS = 1;

// LiDAR debug print rate.
constexpr int ROCK5_LIDAR_LOOKUP_INTERVAL_SECONDS = 1;
#endif

// shared LiDAR lookup limits.
// near noise rejection.
constexpr float LIDAR_MIN_RANGE_M = 0.10F;

// far background rejection.
constexpr float LIDAR_MAX_RANGE_M = 3.00F;

// target direction window.
constexpr float LIDAR_ANGLE_WINDOW_DEG = 4.0F;

// camera y to LiDAR bearing calibration.
constexpr float CAMERA_Y_TO_LIDAR_ANGLE = 56.0F;

// bearing offset from calibration.
constexpr float CAMERA_Y_ANGLE_OFFSET_DEG = -1.0F;

// radian to degree conversion.
constexpr float RAD_TO_DEG = 57.2957795F;


// range gap between separate surfaces(12cm).
constexpr float LIDAR_CLUSTER_GAP_M = 0.12F;

// Minimum support for a valid cluster.
constexpr std::size_t LIDAR_MIN_CLUSTER_POINTS = 2;

// Calibration from Rock 5 lab test data.
// Validated mainly in the 0.30 m to 0.60 m forward range.
constexpr float LIDAR_X_SCALE = 1.00255F;
constexpr float LIDAR_X_OFFSET_M = 0.01556F;

constexpr float LIDAR_Y_SCALE = 1.12269F;
constexpr float LIDAR_Y_OFFSET_M = -0.02852F;

// LiDAR candidate with full geometry.
struct LidarCandidate {
    float x;
    float y;
    float range;
    float angleDeg;
};
}

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


    // camera callbacks are quiescent here, so no new asynchronous
    // disparity or target-detection work can be started.
    stereo.waitUntilIdle();
    targetDet.waitUntilIdle();

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

float TargetLoc::cameraYToLidarAngle(float targetY)
{
    // linear calibration: camera lateral y to LiDAR bearing.
    return CAMERA_Y_TO_LIDAR_ANGLE * targetY + CAMERA_Y_ANGLE_OFFSET_DEG;
}

TargetLoc::LidarTargetEstimate TargetLoc::estimateTargetFromLidar(float targetY)
{
    // empty estimate until a valid cluster is selected.
    LidarTargetEstimate estimate;

    // visual target direction.
    const float targetAngleDeg = cameraYToLidarAngle(targetY);

    // snapshot callback data with short mutex scope.
    std::vector<cv::Point2f> lidarSnapshot;
    {
        std::lock_guard<std::mutex> guard(lidarData_mutex);
        lidarSnapshot = currentLidarCoords;
    }

    // angle-filtered LiDAR candidates.
    std::vector<LidarCandidate> candidates;

    for (const auto &p : lidarSnapshot) {
        // LiDAR robot-frame point.
        const float lidarX = p.x;
        const float lidarY = p.y;

        // camera-visible half-plane only.
        if (lidarX <= 0.0F)
            continue;

        // polar range.
        const float lidarRange = std::hypot(lidarX, lidarY);

        // useful range ring.
        if (lidarRange < LIDAR_MIN_RANGE_M)
            continue;
        if (lidarRange > LIDAR_MAX_RANGE_M)
            continue;

        // LiDAR bearing.
        const float lidarAngleDeg =
            std::atan2(lidarY, lidarX) * RAD_TO_DEG;

        // bearing-window match.
        if (std::fabs(lidarAngleDeg - targetAngleDeg) >
            LIDAR_ANGLE_WINDOW_DEG)
            continue;

        // accepted candidate.
        candidates.push_back({lidarX, lidarY, lidarRange, lidarAngleDeg});
    }

    // candidate count for debug output.
    estimate.candidates = candidates.size();

    // no direction-matched points.
    if (candidates.empty())
        return estimate;

    // nearest range first.
    std::sort(candidates.begin(), candidates.end(),
              [](const LidarCandidate &a, const LidarCandidate &b) {
                  return a.range < b.range;
              });

    // first range cluster.
    std::size_t clusterStart = 0;

    // walk through continuous range clusters.
    while (clusterStart < candidates.size()) {
        std::size_t clusterEnd = clusterStart + 1;

        // same surface while range gap is small.
        while (clusterEnd < candidates.size() &&
               candidates[clusterEnd].range -
                   candidates[clusterEnd - 1].range <=
                   LIDAR_CLUSTER_GAP_M) {
            ++clusterEnd;
        }

        // current cluster size.
        const std::size_t clusterCount = clusterEnd - clusterStart;

        // require enough LiDAR support.
        if (clusterCount >= LIDAR_MIN_CLUSTER_POINTS) {
            // Median point inside selected cluster.
            const std::size_t medianIndex =
                clusterStart + clusterCount / 2;

            // mean bearing for readable debug output.
            float angleSumDeg = 0.0F;
            for (std::size_t i = clusterStart; i < clusterEnd; ++i)
                angleSumDeg += candidates[i].angleDeg;

            // valid selected cluster.
            estimate.valid = true;
            estimate.x = candidates[medianIndex].x;
            estimate.y = candidates[medianIndex].y;
            estimate.range = candidates[medianIndex].range;
            estimate.angleDeg = angleSumDeg / clusterCount;
            estimate.clusterPoints = clusterCount;

            return estimate;
        }

        // next separated range cluster.
        clusterStart = clusterEnd;
    }

    // no supported cluster.
    return estimate;
}

void TargetLoc::updateStereo()
{
    cv::Mat leftSnapshot;
    cv::Mat rightSnapshot;

    {
        std::lock_guard<std::mutex> guard(leftImage_mutex);
        leftSnapshot = currentL;
    }

    {
        std::lock_guard<std::mutex> guard(rightImage_mutex);
        rightSnapshot = currentR;
    }

    // work on snapshots because both camera callbacks can update
    // the stored frames while stereo is being started.
    if (leftSnapshot.empty() || rightSnapshot.empty())
        return;
    if (leftSnapshot.size != rightSnapshot.size)
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
    stereo.calcDepthMapAsync(leftSnapshot, rightSnapshot);
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
    // Detailed mode is intended for targetlocviewer diagnostics.
    const bool detailedOutput = outputMode == OutputMode::Detailed;

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
                if (detailedOutput)
                    fprintf(stderr, "Contour discarded.\n");
                return;
            }
        }
    }

    cv::Mat disparitySnapshot;

    {
        std::lock_guard<std::mutex> guard(disparityData_mutex);
        disparitySnapshot = currentD;
    }

    // Checking if the disparity map is actually there as we need it to find out
    // how far the target is.
    if (disparitySnapshot.empty())
        return;

    // We need to scale the contour from full resolution to the resolution
    // of the disparity map.
    if (detailedOutput) {
        printf("We have a contour around a target:");
    }
    int i = 0;
    float avgX = 0;
    contour_mutex.lock();
    for (auto &c : contour) {
        const int x = c.x * disparitySnapshot.size().width / cameraWidth;
        const int y = c.y * disparitySnapshot.size().height / cameraHeight;
        scaledContour.emplace_back(x, y);
        if (detailedOutput) {
            printf("[%d,%d]", x, y);
        }
        avgX = avgX + c.x;
        i++;
    }
    contour_mutex.unlock();
    avgX = avgX / i;
    if (detailedOutput) {
        printf(", avgX = %f",avgX);
        printf("\n");
    }

    if (detailedOutput) {
        // LiDAR data-path check.
        // copy callback data with short mutex scope.
        std::vector<cv::Point2f> lidarSnapshot;
        {
            std::lock_guard<std::mutex> guard(lidarData_mutex);
            lidarSnapshot = currentLidarCoords;
        }

        // point count during visual detection.
        printf("Latest LIDAR points during detection: %zu\n",
                lidarSnapshot.size());
    }

    // Create mask
    cv::Mat mask = cv::Mat::zeros(disparitySnapshot.size(), CV_8UC1);

    // Draw filled contour for the mask
    std::vector<std::vector<cv::Point>> scaledContours{scaledContour};
    cv::drawContours(mask, scaledContours, -1, cv::Scalar(255), cv::FILLED);

    // Compute mean disparity value inside contour
    float avgDisp = cv::mean(disparitySnapshot, mask)[0];

    cv::Point2f targetLoc;

    // mapping disparity to distance in meter for the LIDAR / egocentric
    // x-coordinate
    targetLoc.x = disp2meter / avgDisp;

    // mapping the x-coordinate pixels of the camera to the LIDAR y-coordinate
    // of the target in meter
    targetLoc.y = xpos2meter * (xposAtCentre - avgX);

#ifdef TARGETLOC_USE_ROCK5_V4L_CAMERA
    // LiDAR debug rate limit on rock5.
    // target detection can run faster than this lookup needs.
    static auto lastLidarLookupTime =
        std::chrono::steady_clock::now() -
        std::chrono::seconds(ROCK5_LIDAR_LOOKUP_INTERVAL_SECONDS);

    // current lookup time.
    const auto lidarLookupNow = std::chrono::steady_clock::now();

    // run one LiDAR lookup window per second on rock5.
    const bool runLidarLookup = 
        lidarLookupNow - lastLidarLookupTime >=
        std::chrono::seconds(ROCK5_LIDAR_LOOKUP_INTERVAL_SECONDS);
    
    // update Rock 5 lookup timestamp.
    if (runLidarLookup)
        lastLidarLookupTime = lidarLookupNow;
#else
    const bool runLidarLookup = true;
#endif

    // ===================**LiDAR estimate rate.**==========
    // camera-derived y is only used as a direction cue for LiDAR lookup.
    const float cameraTargetY = targetLoc.y;

    // on Rock 5, LiDAR lookup is throttled.
    // do not output stereo-only coordinates between LiDAR lookup ticks.
    if (!runLidarLookup){
        return;
    } else {
        // estimate target coordinate from LiDAR using the camera direction cue.
        const LidarTargetEstimate lidarEstimate =
            estimateTargetFromLidar(cameraTargetY);

        if (lidarEstimate.valid) {

            // final robot-frame coordinate from calibrated LiDAR estimate.
            const float calibratedLidarX =
                LIDAR_X_SCALE * lidarEstimate.x + LIDAR_X_OFFSET_M;
            const float calibratedLidarY =
                LIDAR_Y_SCALE * lidarEstimate.y + LIDAR_Y_OFFSET_M;

            targetLoc.x = calibratedLidarX;
            targetLoc.y = calibratedLidarY;

            // Raw cluster details are useful in viewer diagnostics only.
            if (detailedOutput) {
                // selected LiDAR target cluster.
                printf("LiDAR target: camera_y=%f, raw_lidar_x=%f, raw_lidar_y=%f, "
                    "calibrated_x=%f, calibrated_y=%f, range=%f, angle=%f, "
                    "candidates=%zu, cluster_points=%zu\n",
                    cameraTargetY,
                    lidarEstimate.x,
                    lidarEstimate.y,
                    calibratedLidarX,
                    calibratedLidarY,
                    lidarEstimate.range,
                    lidarEstimate.angleDeg,
                    lidarEstimate.candidates,
                    lidarEstimate.clusterPoints);
            }
        } else {
            // A visual target exists, but no final coordinate can be produced.
            if (detailedOutput) {
                // no valid LiDAR cluster in target direction.
                printf("LiDAR target: no valid estimate, camera_y=%f, candidates=%zu\n",
                   cameraTargetY,
                   lidarEstimate.candidates);
            } else {
                printf("TargetLoc: visual target detected, "
                       "waiting for a valid LiDAR estimate.\n");
            }
            return;
        }
    }

    if (detailedOutput) {
        printf("TargetLoc: visual target detected, "
               "disparity=%f, target_loc=[%f,%f]\n",
               avgDisp, targetLoc.x, targetLoc.y);
    } else {
        // Report the final calibrated LiDAR coordinate.
        printf("TargetLoc final coordinate: x=%.3f m, y=%.3f m\n",
               targetLoc.x, targetLoc.y);
    }

    // callback!
    if (detectionInterface) {
        detectionInterface->newTargetDetected(targetLoc);
    }
}
