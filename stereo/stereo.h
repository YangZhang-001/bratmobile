#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <thread>
#include <atomic>

#pragma once

class Stereo
{
public:
    static inline cv::Mat convertColour2Grey(const cv::Mat& colourImage) {
        cv::Mat greyImage;
	    cv::cvtColor(colourImage, greyImage, cv::COLOR_BGR2GRAY);
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
    void registerCallback(OnDisparity cb) {
        onDisparity = cb;
    }

    // Camera matrices
    cv::Mat K1,
        D1, K2, D2;
    cv::Mat R, T, E, F;

    // Rectification
    cv::Mat R1, R2, P1, P2, Q;

    // Maps: raw->rect images
    cv::Mat map1L, map2L, map1R, map2R;

    // checker board points
    std::vector<std::vector<cv::Point2f>> imgPointsLeft;
    std::vector<std::vector<cv::Point2f>> imgPointsRight;

    bool hasValidCalibration = false;
    bool isCalibrating = false;

    ~Stereo();

private:
    void calcMaps();
        // Stereo matching
    cv::Ptr<cv::StereoSGBM> stereoMatcher = cv::StereoSGBM::create(
        0, 16 * 5, 5
    );
    cv::Size imageSize{0,0};

    std::thread disparityCalcThread;
    std::atomic<bool> isCalculatingDisparity = false;
    OnDisparity onDisparity;
};
