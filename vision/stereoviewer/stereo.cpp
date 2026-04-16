#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "stereo.h"

int Stereo::calibration(const std::vector<cv::Mat> leftImages,
                        const std::vector<cv::Mat> rightImages)
{
    cv::Size boardSize(9, 6);
    float squareSize = 0.025f;

    std::vector<std::vector<cv::Point3f>> objectPoints;

    imgPointsLeft.clear();
    imgPointsRight.clear();

    // Prepare object points
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    cv::Size imageSize;

    // Detect corners
    for (size_t i = 0; i < leftImages.size(); i++)
    {
        cv::Mat left = leftImages[i];
        cv::Mat right = rightImages[i];

        if (left.empty() || right.empty())
        {
            std::cout << "Error loading images\n";
            return -1;
        }

        imageSize = left.size();

        std::vector<cv::Point2f> cornersL, cornersR;

        bool foundL = cv::findChessboardCorners(left, boardSize, cornersL);
        bool foundR = cv::findChessboardCorners(right, boardSize, cornersR);

        if (foundL && foundR)
        {
            cv::cornerSubPix(left, cornersL, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

            cv::cornerSubPix(right, cornersR, cv::Size(11, 11), cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01));

            imgPointsLeft.push_back(cornersL);
            imgPointsRight.push_back(cornersR);
            objectPoints.push_back(obj);

            std::cout << "Corners detected for pair " << i << "\n";
        }
    }

    // Stereo calibration
    cv::stereoCalibrate(
        objectPoints,
        imgPointsLeft,
        imgPointsRight,
        K1, D1,
        K2, D2,
        imageSize,
        R, T, E, F);

    std::cerr << "Calibration done\n";

    cv::stereoRectify(
        K1, D1,
        K2, D2,
        imageSize,
        R, T,
        R1, R2,
        P1, P2,
        Q);
        return 0;
}

int Stereo::saveCal(const std::string calPath)
{

    cv::FileStorage fs(calPath, cv::FileStorage::WRITE);

    fs << "K1" << K1;
    fs << "D1" << D1;
    fs << "K2" << K2;
    fs << "D2" << D2;

    fs << "R1" << R1;
    fs << "R2" << R2;
    fs << "P1" << P1;
    fs << "P2" << P2;
    fs << "Q" << Q;

    fs.release();
    return 0;
}

int Stereo::loadCal(const std::string calPath)
{
    cv::FileStorage fs(calPath, cv::FileStorage::READ);

    fs["K1"] >> K1;
    fs["D1"] >> D1;
    fs["K2"] >> K2;
    fs["D2"] >> D2;

    fs["R1"] >> R1;
    fs["R2"] >> R2;
    fs["P1"] >> P1;
    fs["P2"] >> P2;
    fs["Q"] >> Q;

    fs.release();
    return 0;
}