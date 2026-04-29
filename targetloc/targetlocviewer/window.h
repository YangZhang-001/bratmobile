#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>

#include <opencv2/core/types.hpp>
#include <qcustomplot.h>

#include "targetloc.h"


class QLIDARPlot:public QCustomPlot
{
public:
    QLIDARPlot():QCustomPlot(){
	QSizePolicy p(sizePolicy());
        p.setHeightForWidth(true);
        setSizePolicy(p);
    };
    virtual int heightForWidth ( int w ) const override { return w;};
};

// class definition 'Window'
class Window : public QWidget, public TargetLoc::DetectionInterface
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    ~Window()
    {
        targetLoc.stop();
        lidar.stop();
    }

private:
    QHBoxLayout *hLayout;
    QVBoxLayout *vLayout;

    QLabel *imageCombined;
    QLabel *imageDisparity;

    void updateGUI();

    void timerEvent(QTimerEvent *event);

    TargetLoc targetLoc;

    C1Lidar lidar;

    const cv::Size displayImageSize{640, 360};

    QLIDARPlot *lidarPlot;

    virtual void newTargetDetected(const cv::Point2f topViewEgoCoordinate);
};

#endif // WINDOW_H
