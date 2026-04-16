#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include "libcam2opencv.h"

#include "stereo.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    void updateImageL(const cv::Mat &mat);
    void updateImageR(const cv::Mat &mat);

private:
    QHBoxLayout  *h1Layout;
    QHBoxLayout  *h2Layout;
    QVBoxLayout  *vLayout;
    QLabel       *imageL;
    QLabel       *imageR;
    QLabel       *imageCombined;

    QPushButton  *calibratePushbutton;
    QLabel       *calInfo;

    static constexpr int displaywidth = 640;
    static constexpr int numFrames4Calibration = 60;

    std::vector<cv::Mat> leftImages4Cal;
    std::vector<cv::Mat> rightImages4Cal;

    cv::Mat currentL;
    cv::Mat currentR;

    Stereo stereo;

    // this clears our image pairs and cellects numFrames4Calibration frames
    void triggerCalibration();

    // checks if we have all frame for calibraton and starts it
    void check4Cal();

    // blends the current L and R images and displays it
    void blendLR();

    std::thread calThread;

    bool calibrationTriggered = false;
};

#endif // WINDOW_H
