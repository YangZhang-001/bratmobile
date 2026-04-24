#include "targetloc.h"
#include <libcamera/libcamera/camera_manager.h>

void TargetLoc::start()
{
    cm.start();

    cameraL.registerCallback([&](const cv::Mat &left, const libcamera::ControlList &)
                             { updateImageL(left); });

    cameraR.registerCallback([&](const cv::Mat &right, const libcamera::ControlList &)
                             { updateImageR(right); });

    targetDet.registerDetCallback([&](const std::vector<cv::Point2f> coord){onTargetDetected(coord);});

    stereo.registerCallback([&](cv::Mat d){currentD = d;});

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
}

void TargetLoc::updateStereo()
{
    stereo.calcDepthMapAsync(currentL,currentR);
}

void TargetLoc::updateImageL(const cv::Mat& l)
{
    currentL = l;
    updateStereo();
    targetDet.detectAsync(l);
}

void TargetLoc::updateImageR(const cv::Mat& r)
{
    currentR = r;
    updateStereo();
}

// here it's where it's getting interesting!
void TargetLoc::onTargetDetected(std::vector<cv::Point2f> coord) {

}
