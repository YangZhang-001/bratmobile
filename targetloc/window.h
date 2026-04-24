#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include "targetloc.h"

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    ~Window() {
        targetLoc.stop();
    }

private:
    QHBoxLayout  *h1Layout;
    QHBoxLayout  *h2Layout;
    QVBoxLayout  *vLayout;
    QLabel       *imageL;
    QLabel       *imageR;
    QLabel       *imageCombined;
    QLabel       *imageDisparity;

    void updateGUI();

    void timerEvent(QTimerEvent *event);

    TargetLoc targetLoc;

    const cv::Size displayImageSize{640,360};
};

#endif // WINDOW_H
