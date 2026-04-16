#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "stereo.h"

int Stereo::calibration(const std::vector<cv::Mat> leftImages,
                        const std::vector<cv::Mat> rightImages)
{
    if (isCalibrating)
    {
        fprintf(stderr, "calibration is called while another cal thread is running.\n");
        return -1;
    }

    isCalibrating = true;

    cv::Size boardSize(9, 6);
    float squareSize = 0.025f;

    std::vector<std::vector<cv::Point3f>> objectPoints;

    imgPointsLeft.clear();
    imgPointsRight.clear();

    fprintf(stderr, "Prepare object points\n");
    std::vector<cv::Point3f> obj;
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            obj.push_back(cv::Point3f(j * squareSize, i * squareSize, 0));
        }
    }

    int nCorners = 0;

    fprintf(stderr, "Detect corners.\n");
    for (size_t i = 0; i < leftImages.size(); i++)
    {
        cv::Mat left = leftImages[i];
        cv::Mat right = rightImages[i];

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
            nCorners++;
        }
    }

    if (0 == nCorners)
    {
        fprintf(stderr, "No corners detected!\n");
        isCalibrating = false;
        hasValidCalibration = false;
        return -1;
    }

    fprintf(stderr, "Stereo calibration.\n");
    cv::stereoCalibrate(
        objectPoints,
        imgPointsLeft,
        imgPointsRight,
        K1, D1,
        K2, D2,
        imageSize,
        R, T, E, F);

    fprintf(stderr, "Stereo rectify.\n");
    cv::stereoRectify(
        K1, D1,
        K2, D2,
        imageSize,
        R, T,
        R1, R2,
        P1, P2,
        Q);

    isCalibrating = false;
    hasValidCalibration = true;

    fprintf(stderr, "Stereo calibration finished.\n");

    calcMaps();

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
    fs << "imageSize" << imageSize;

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
    fs["imageSize"] >> imageSize;

    fs.release();

    calcMaps();
    return 0;
}

void Stereo::calcMaps()
{
    cv::initUndistortRectifyMap(K1, D1, R1, P1, imageSize, CV_16SC2, map1L, map2L);
    cv::initUndistortRectifyMap(K2, D2, R2, P2, imageSize, CV_16SC2, map1R, map2R);
}

cv::Mat Stereo::rectifyLeft(const cv::Mat &left)
{
    if (map1L.empty() || map2L.empty())
        return left;
    cv::Mat rectLeft;
    cv::remap(left, rectLeft, map1L, map2L, cv::INTER_LINEAR);
    return rectLeft;
}

cv::Mat Stereo::rectifyRight(const cv::Mat &right)
{
    if (map1R.empty() || map2R.empty())
        return right;
    cv::Mat rectRight;
    cv::remap(right, rectRight, map1R, map2R, cv::INTER_LINEAR);
    return rectRight;
}

cv::Mat Stereo::calcDepthMapSync(const cv::Mat &left, const cv::Mat &right)
{
 //   fprintf(stderr,"Disp calc start.\n");
    cv::Mat rectLeft = rectifyLeft(left);
    cv::Mat rectRight = rectifyRight(right);

    stereoMatcher->setP1(8 * rectLeft.channels() * 5 * 5);
    stereoMatcher->setP2(32 * rectLeft.channels() * 5 * 5);

    cv::Mat disparity;
    stereoMatcher->compute(rectLeft, rectRight, disparity);
//    fprintf(stderr,"Disp calc finished.\n");
    return disparity;
}

void Stereo::calcDepthMapAsync(const cv::Mat &left, const cv::Mat &right)
{
    if (isCalculatingDisparity)
        return;
    if (disparityCalcThread.joinable())
    {
        disparityCalcThread.join();
    }
    disparityCalcThread = std::thread([&]()
                                        {     
                                        isCalculatingDisparity = true;
                                        const cv::Mat d = calcDepthMapSync(left, right); 
                                        onDisparity(d);     
                                        isCalculatingDisparity = false; 
                                        });
}

Stereo::~Stereo()
{
    if (disparityCalcThread.joinable())
    {
        disparityCalcThread.join();
    }
}