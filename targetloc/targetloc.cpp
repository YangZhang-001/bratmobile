#include "targetloc.h"
#include <libcamera/libcamera/camera_manager.h>

void TargetLoc::start()
{
    cm.start();

    cameraL.registerCallback([&](const cv::Mat &left, const libcamera::ControlList &)
                             { updateImageL(left); });

    cameraR.registerCallback([&](const cv::Mat &right, const libcamera::ControlList &)
                             { updateImageR(right); });

    targetDet.registerDetCallback([&](const cv::Point2f coord)
                                  { onTargetDetected(coord); });

    stereo.registerCallback([&](cv::Mat d)
                            { currentD = d; });

    Libcam2OpenCVSettings settings;
    settings.width = 1920;
    settings.height = 1080;
    settings.cameraIndex = 0;
    cameraL.start(cm, settings);
    settings.cameraIndex = 1;
    cameraR.start(cm, settings);
}

void TargetLoc::stop()
{
    cameraL.stop();
    cameraR.stop();
    cm.stop();
}

void TargetLoc::newScanAvail(C1LidarData (&data)[C1Lidar::nDistance])
{
    currentLidarCoords.clear();
    for(const auto& v:data) {
        currentLidarCoords.push_back({v.x,v.y});
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
    // yes, we have!
    stereo.calcDepthMapAsync(currentL, currentR);
}

void TargetLoc::updateImageL(const cv::Mat &l)
{
    currentL = l;
    updateStereo();
    // We detect the target from the left eye!
    targetDet.detectAsync(l);
}

void TargetLoc::updateImageR(const cv::Mat &r)
{
    currentR = r;
    updateStereo();
}

// here it's where it's getting interesting!
void TargetLoc::onTargetDetected(const cv::Point2f coord)
{
    printf("Target/camera: %f,%f\n",coord.x,coord.y);    
}
