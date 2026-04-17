#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

class TargetDet
{
public:
    TargetDet();
    ~TargetDet();
    std::shared_ptr<cv::QRCodeDetector> qrDetector;

    // callback for the coordinate
    using OnDetected = std::function<void(cv::Point2f)>;

    std::vector<cv::Point2f> detectSync(const cv::Mat img);

    void detectAsync(const cv::Mat img);

    void registerDetCallback(OnDetected cb) {
        onDetected = cb;
    }

    std::atomic<bool> isDetecting = false;

private:

    OnDetected onDetected;
    std::thread detThread;
};
