#pragma once

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>

#include <iostream>
#include <string>
#include <thread>
#include <functional>
#include <atomic>
#include <vector>


/**
 * Raw V4L2 control parameter.
 * The device path can be a video node or a subdevice such as /dev/v4l-subdev2.
 */
struct V4L2ControlParameter
{
    std::string devicePath;
    int parameter;
    float value;
};

/**
 * OpenCV/V4L2 capture parameters.
 */
struct V4L2OpenCVParameters
{
    int deviceID = 0;
    unsigned int fourcc = 0;
    int width = 0;
    int height = 0;
    int framerate = 0;
};

/**
 * Minimal OpenCV/V4L2 camera backend used on Rock 5.
 * Mirror the callback style of the existing libcamera path.
 */
class V4L2Camera
{
  public:
    using OnFrame = std::function<void (const cv::Mat &)>;

    V4L2Camera () = default;

    V4L2OpenCVParameters
    start (const V4L2OpenCVParameters openCVparameters = V4L2OpenCVParameters (),
           const std::vector<V4L2ControlParameter> v4lParameters = {});

    void stop ();

    void
    registerFrameCallback (OnFrame cb)
    {
        onFrame = cb;
    }

  private:
    void threadLoop ();
    
    cv::VideoCapture videoCapture;
    std::thread cameraThread;
    std::atomic<bool> isOn = false;
    OnFrame onFrame;

    void setV4Lparameter (const V4L2ControlParameter &v4lParameter);
};