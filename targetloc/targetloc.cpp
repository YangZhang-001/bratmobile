#include "targetloc.h"
#include <libcamera/libcamera/camera_manager.h>

void TargetLoc::start()
{
    cm.start();

    cameraL.registerCallback([&](const cv::Mat &left, const libcamera::ControlList &)
                             { updateImageL(left); });

    cameraR.registerCallback([&](const cv::Mat &right, const libcamera::ControlList &)
                             { updateImageR(right); });

    targetDet.registerDetCallback([&](const std::vector<cv::Point2f> &coords)
                                  { onTargetDetected(coords); });

    stereo.registerCallback([&](cv::Mat d) {
	std::lock_guard<std::mutex> guard(disparityData_mutex);
	currentD = d; 
    });

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
    std::lock_guard<std::mutex> guard(lidarData_mutex);
    currentLidarCoords.clear();
    for (const auto &v : data)
    {
        if (v.valid)
        {
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
    printf("Contour: ");
    int i = 0;
    float avgX = 0;
    contour_mutex.lock();
    scaledContour.clear();
    for(auto &c:contour) {
	const int x = c.x * currentD.size().width / settings.width;
	const int y = c.y * currentD.size().height / settings.height;
	scaledContour.emplace_back(x,y);
	printf("[%d,%d]", x, y);
	avgX = avgX + c.x;
	i++;
    }
    contour_mutex.unlock();    
    avgX = avgX / i;
    printf("\n");

    if (currentD.empty()) return;

    // Create mask
    cv::Mat mask = cv::Mat::zeros(currentD.size(), CV_8UC1);
    std::cout << mask.size() << std::endl;

    // Draw filled contour for the mask
    std::vector<std::vector<cv::Point>> scaledContours{scaledContour};
    cv::drawContours(mask, scaledContours,-1, cv::Scalar(255), cv::FILLED);

    // Compute mean gray value inside contour
    float avgDisp = cv::mean(currentD, mask)[0];

    const float targetLocX = disp2meter / avgDisp;
    const float targetLocY = xpos2meter * ((settings.width/2) - avgX);

    printf("Disparity: %f, Target location: [%f,%f]\n",avgDisp,targetLocX,targetLocY);

    // callback!
    if (detectionInterface)
    {
        detectionInterface->newTargetDetected(targetLocX, targetLocY);
    }
}
