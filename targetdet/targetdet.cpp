#include "targetdet.h"

TargetDet::TargetDet()
{
}

TargetDet::~TargetDet()
{
    if (detThread.joinable()) {
        detThread.join();
    }
}

std::vector<cv::Point2f> TargetDet::detectSync(const cv::Mat img)
{
    std::vector<cv::Point2f> points;
    if (img.empty())
    {
        fprintf(stderr, "TargetDetSync: image empty\n");
        return points;
    }
    qrDetector->detect(img.clone(), points);
    return points;
}

void TargetDet::detectAsync(const cv::Mat img)
{
    if (img.empty())
    {
        fprintf(stderr, "TargetDetASync: image empty\n");
        return;
    }
    if (isDetecting)
        return;
    if (detThread.joinable())
    {
        detThread.join();
    }
    detThread = std::thread([&](cv::Mat imgThr)
                            {
                            isDetecting=true;
                            auto pt = detectSync(imgThr);
                            if (pt.size()>0)
                                onDetected(pt[0]);
                            isDetecting=false; },img);
}
