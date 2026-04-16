#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#pragma once

struct Stereo
{
    int calibration(const std::vector<cv::Mat> leftImages,
                    const std::vector<cv::Mat> rightImages);

    int saveCal(const std::string calPath);

    int loadCal(const std::string calPath);

    // Camera matrices
    cv::Mat K1, D1, K2, D2;
    cv::Mat R, T, E, F;

    // Rectification
    cv::Mat R1, R2, P1, P2, Q;

    std::vector<std::vector<cv::Point2f>> imgPointsLeft;
    std::vector<std::vector<cv::Point2f>> imgPointsRight;

    bool hasValidCalibration = false;
    bool isCalibrating = false;
};
