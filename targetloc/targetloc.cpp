#include "targetloc.h"

void TargetLoc::start()
{
    	Libcam2OpenCV cameraL;
	cameraL.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							 { 
								window.updateImageL(mat); });

	Libcam2OpenCV cameraR;
	cameraR.registerCallback([&](const cv::Mat &mat, const libcamera::ControlList &)
							 { 
								window.updateImageR(mat); });

	Libcam2OpenCVSettings settings;
	settings.width=1920;
	settings.height=1080;
	settings.cameraIndex = 0;
	cameraL.start(cm, settings);
	settings.cameraIndex = 1;
	cameraR.start(cm, settings);
}

void TargetLoc::stop()
{
}
