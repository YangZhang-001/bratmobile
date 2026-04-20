#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>
#include "PrimeStereoMatch.h"

#pragma once

class Stereo
{
public:
    static inline cv::Mat convertColour2Grey(const cv::Mat &colourImage)
    {
        cv::Mat greyImage;
        cv::cvtColor(colourImage, greyImage, cv::COLOR_BGR2GRAY);
        cv::resize(greyImage, greyImage, cv::Size(640, 360));
        return greyImage;
    }

    int calibration(const std::vector<cv::Mat> leftImages,
                    const std::vector<cv::Mat> rightImages);

    // Saves the camera parameters
    int saveCal(const std::string calPath);

    // Loads the camera parameters
    int loadCal(const std::string calPath);

    // un-distorts the left camera image
    cv::Mat rectifyLeft(const cv::Mat &left);

    // un-distorts the right camera image
    cv::Mat rectifyRight(const cv::Mat &right);

    // blocking call to calc depth map
    cv::Mat calcDepthMapSync(const cv::Mat &left, const cv::Mat &right);

    // callback for the disparity
    using OnDisparity = std::function<void(const cv::Mat &)>;

    // runs in a thread and only calcs a new map if no thread is running
    void calcDepthMapAsync(const cv::Mat &left, const cv::Mat &right);

    // registers callback
    void registerCallback(OnDisparity cb)
    {
        onDisparity = cb;
    }

    ~Stereo();

private:
    // Stereo matching
    cv::Ptr<cv::StereoSGBM> stereoMatcher = cv::StereoSGBM::create(
        0,      // minDisp
        16 * 5, // numDisp,
        3       // block size
    );
    cv::Size imageSize{0, 0};

    std::thread disparityCalcThread;
    std::atomic<bool> isCalculatingDisparity = false;
    OnDisparity onDisparity;
};
