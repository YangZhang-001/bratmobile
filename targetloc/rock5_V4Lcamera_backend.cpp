#include "rock5_V4Lcamera_backend.h"
#include <algorithm>  //std::max

#include <cmath>

V4L2Camera::~V4L2Camera ()
{
    stop ();
}

void V4L2Camera::threadLoop ()
{
    //isOn = true;
    while (isOn) 
    {
        cv::Mat cap;

        videoCapture.set (cv::CAP_PROP_CONVERT_RGB, true);
        // read a new frame from camera
        videoCapture.read (cap);
        if (cap.empty ()) 
        {
            std::cerr << "ERROR! blank frame grabbed\n";
            isOn = false;
            return;
        }

        // the rock5 test pattern has correct colors
        // the image on rock5 platform is green when the ISP/AWB pipeline is not used,
        // so apply a lightweight grey-world white balance
        if (cap.channels () == 3)
        {
            //store B/G/R channels in "channels"
            std::vector<cv::Mat> channels;
            cv::split (cap, channels);

            const double meanB = cv::mean (channels[0])[0];
            const double meanG = cv::mean (channels[1])[0];
            const double meanR = cv::mean (channels[2])[0];

            const double gray = (meanB + meanG + meanR) / 3.0;

            channels[0].convertTo (channels[0], -1, gray / std::max (meanB, 1.0));
            channels[1].convertTo (channels[1], -1, gray / std::max (meanG, 1.0));
            channels[2].convertTo (channels[2], -1, gray / std::max (meanR, 1.0));

            cv::merge (channels, cap);

            // slightly increase contrast and brightness
            cap.convertTo (cap, -1, 1.25, 8.0);
        }

        if (onFrame) 
        {
            onFrame (cap);
        }
    }
}

void V4L2Camera::setV4Lparameter (const V4L2ControlParameter &v4lParameter)
{
    int fd = open (v4lParameter.devicePath.c_str (), O_RDWR);
    if (fd < 0) {
        perror ("Opening video device");
        return;
    }

    struct v4l2_queryctrl query;
    query.id = v4lParameter.parameter;
    if (ioctl (fd, VIDIOC_QUERYCTRL, &query) == 0) 
    {
        int d = query.maximum - query.minimum;
        struct v4l2_control control;
        control.id = v4lParameter.parameter;
        control.value = query.minimum + (int)round (v4lParameter.value * d);
        if (control.value > query.maximum)
            control.value = query.maximum;
        if (control.value < query.minimum)
            control.value = query.minimum;
        if (ioctl (fd, VIDIOC_S_CTRL, &control) < 0) 
        {
            perror ("Setting Parameter");
        }
    } 
    else 
    {
        perror ("Querying video device");
        std::cerr << v4lParameter.devicePath << "," << v4lParameter.parameter
                  << "," << v4lParameter.value << std::endl;
    }

    close (fd);
}

V4L2OpenCVParameters V4L2Camera::start (V4L2OpenCVParameters openCVparameters,
                   const std::vector<V4L2ControlParameter> v4lParameters)
{
    for (const auto &p : v4lParameters) {
        setV4Lparameter (p);
    }

    videoCapture.open (openCVparameters.deviceID, cv::CAP_V4L2);
    if (!videoCapture.isOpened ()) {
        std::cerr << "ERROR! cannot open /dev/video" << openCVparameters.deviceID << "\n";
        return openCVparameters;
    }

    if (openCVparameters.fourcc > 0) {
        videoCapture.set (cv::CAP_PROP_FOURCC, openCVparameters.fourcc);
    }
    if ((openCVparameters.width > 0) && (openCVparameters.height > 0)) 
    {
        videoCapture.set (cv::CAP_PROP_FRAME_WIDTH, openCVparameters.width);
        videoCapture.set (cv::CAP_PROP_FRAME_HEIGHT, openCVparameters.height);
    }
    if (openCVparameters.framerate > 0) {
        videoCapture.set (cv::CAP_PROP_FPS, openCVparameters.framerate);
    }

    videoCapture.set (cv::CAP_PROP_CONVERT_RGB, 1);

    openCVparameters.width = videoCapture.get (cv::CAP_PROP_FRAME_WIDTH);
    openCVparameters.height = videoCapture.get (cv::CAP_PROP_FRAME_HEIGHT);
    openCVparameters.fourcc = videoCapture.get (cv::CAP_PROP_FOURCC);
    openCVparameters.framerate = videoCapture.get (cv::CAP_PROP_FPS);

    if ((openCVparameters.height > 0) && (openCVparameters.width > 0)) 
    {
        // Set this before creating the thread so an immediate stop cannot
        // be overwritten when threadLoop starts.
        isOn = true;
        cameraThread = std::thread (&V4L2Camera::threadLoop, this);
    }
    else
    {
        isOn = false;
    }

    return openCVparameters;
}

void
V4L2Camera::stop ()
{
    isOn = false;
    if (cameraThread.joinable ()) {
        cameraThread.join ();
    }
    if (videoCapture.isOpened ()) {
        videoCapture.release ();
    }
}
