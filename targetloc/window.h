#ifndef WINDOW_H
#define WINDOW_H

#include <QBoxLayout>
#include <QPushButton>
#include <QLabel>
#include <QPushButton>
#include <QtCharts/QChartView>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QValueAxis>

#include "targetloc.h"

const char RPI_SERIAL_DEV* = "/dev/ttyAMA0";

// class definition 'Window'
class Window : public QWidget
{
    // must include the Q_OBJECT macro for the Qt signals/slots framework to work with this class
    Q_OBJECT

public:
    Window();
    ~Window() {
        targetLoc.stop();
        lidar.stop();
    }

private:
    QHBoxLayout  *h2Layout;
    QVBoxLayout  *vLayout;

    QLabel       *imageCombined;
    QLabel       *imageDisparity;

    void updateGUI();

    void timerEvent(QTimerEvent *event);

    TargetLoc targetLoc;

	C1Lidar lidar;

    const cv::Size displayImageSize{640,360};

    QChart *chart;
    QChartView *chartView;
    QScatterSeries *series;
};

#endif // WINDOW_H
