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

    stereo.registerCallback([&](cv::Mat d)
                            { 
                                disparityData_mutex.lock();
                                currentD = d; 
                                disparityData_mutex.unlock(); });

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
    float targetLocX = 0;
    float targetLocY = 0;
    std::vector<cv::Point> scaledContour;
    printf("Contour: ");
    int i = 0;
    for(auto &c:contour) {
	const int x = c.x * currentD.size().width / settings.width;
	const int y = c.y * currentD.size().height / settings.height;
	scaledContour.emplace_back(x,y);
	printf("[%d,%d]", x, y);
	targetLocX = targetLocX + c.x;
	i++;
    }
    targetLocX = targetLocX / i;
    printf("\n");

    if (currentD.empty()) return;

    // Create mask
    cv::Mat mask = cv::Mat::zeros(currentD.size(), CV_8UC1);
    std::cout << mask.size() << std::endl;

    // Draw filled contour
    std::vector<std::vector<cv::Point>> scaledContours{scaledContour};
    cv::drawContours(mask, scaledContours,-1, cv::Scalar(255), cv::FILLED);

    // Compute mean gray value inside contour
    float avgDisp = cv::mean(currentD, mask)[0];

    targetLocY = disp2meter / avgDisp;

    printf("Disparity: %f, Target location: [%f,%f]\n",avgDisp,targetLocX,targetLocY);
}
