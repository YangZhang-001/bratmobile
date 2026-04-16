#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

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
    cv::Mat rectifyLeft(cv::Mat &left);

    // un-distorts the right camera image
    cv::Mat rectifyRight(cv::Mat &right);

    cv::Mat calcDepthMap(cv::Mat &left, cv::Mat &right);

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

private:
    void calcMaps();
        // Stereo matching
    cv::Ptr<cv::StereoSGBM> stereoMatcher = cv::StereoSGBM::create(
        0, 16 * 5, 5
    );
    cv::Size imageSize{0,0};
};
