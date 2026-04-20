#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "stereo.h"

void Stereo::start(cv::Size inputImageSize, StereoAlgo algo)
{
    stereoAlgo = algo;
    imageSize = inputImageSize;
    stereoMatcher = cv::StereoSGBM::create(
        0,      // minDisp
        16 * 5, // numDisp,
        3       // block size
    );
    primeStereoMatch = std::make_shared<PrimeStereoMatch>(inputImageSize);
}

cv::Mat Stereo::rectifyLeft(const cv::Mat &left)
{
    return left;
}

cv::Mat Stereo::rectifyRight(const cv::Mat &right)
{
    return right;
}

cv::Mat Stereo::calcDepthMapSync(const cv::Mat &left, const cv::Mat &right)
{
    cv::Mat disparity;
    switch (stereoAlgo)
    {
    case OpenCVStereo:
        stereoMatcher->setP1(8 * left.channels() * 5 * 5);
        stereoMatcher->setP2(32 * left.channels() * 5 * 5);
        stereoMatcher->compute(left, right, disparity);
        break;
    case PrimeStereo:
        primeStereoMatch->setInputImages(left, right);
        primeStereoMatch->process();
        disparity = primeStereoMatch->getDisp();
        break;
    }
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
    disparityCalcThread = std::thread([&](const cv::Mat &leftThr, const cv::Mat &rightThr)
                                      {     
                                        isCalculatingDisparity = true;
                                        const cv::Mat d = calcDepthMapSync(leftThr, rightThr);
					if (onDisparity) {
					    onDisparity(d);
					}
                                        isCalculatingDisparity = false; }, left, right);
}

Stereo::~Stereo()
{
    if (disparityCalcThread.joinable())
    {
        disparityCalcThread.join();
    }
}
