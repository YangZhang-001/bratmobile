#include "targetloc.h"

void TargetLoc::start()
{
    cm.start();

    cameraL.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
                             { updateImageL(mat); });

    cameraR.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
                             { updateImageR(mat); });

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

void TargetLoc::updateImageL(cv::Mat& l)
{
}

void TargetLoc::updateImageR(cv::Mat& r)
{
}
