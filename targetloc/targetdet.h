#pragma once

#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>
#include <vector>

class TargetDet
{
public:
    ~TargetDet();
    /**
    * Blocks until the current asynchronous detection has completed.
     * Frame callbacks must be stopped before this function is called.
     */
    void waitUntilIdle();

    cv::QRCodeDetector qrDetector;
    cv::barcode::BarcodeDetector barcodeDetector;

    enum DetectorType
    {
        QR,
        Barcode
    };

    void setDetectorType(DetectorType dt)
    {
        detectorType = dt;
    }

    // callback for the coordinate
    using OnDetected = std::function<void(const std::vector<cv::Point2f>&)>;

    const std::vector<cv::Point2f> detectSync(const cv::Mat img);

    void detectAsync(const cv::Mat img);

    void registerDetCallback(OnDetected cb)
    {
        onDetected = cb;
    }

    cv::Point2f calcCentre(std::vector<cv::Point2f>) const;

    std::atomic<bool> isDetecting = false;

private:
    DetectorType detectorType = QR;
    OnDetected onDetected;
    std::thread detThread;
};
