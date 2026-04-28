#include "window.h"
#include <c1lidarrpi.h>

Window::Window()
{
    hLayout = new QHBoxLayout();

    lidarPlot = new QLIDARPlot;
    lidarPlot->addGraph();
    lidarPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
    lidarPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
    lidarPlot->xAxis->setRange(-5, 5);
    lidarPlot->yAxis->setRange(-5, 5);

    lidarPlot->addGraph();
    lidarPlot->graph(1)->setLineStyle(QCPGraph::lsNone);
    lidarPlot->graph(1)->setScatterStyle(QCPScatterStyle::ssDiamond);
    QPen pen;
    pen.setColor(QColor(255, 0, 0));
    pen.setWidth(5);
    lidarPlot->graph(1)->setPen(pen);

    hLayout->addWidget(lidarPlot);

    vLayout = new QVBoxLayout();

    imageDisparity = new QLabel;
    vLayout->addWidget(imageDisparity);

    imageCombined = new QLabel;
    vLayout->addWidget(imageCombined);

    hLayout->addLayout(vLayout);

    setLayout(hLayout);

    fprintf(stderr, "Starting screen update timer.\n");
    startTimer(std::chrono::milliseconds{100});

    fprintf(stderr, "Starting Targetloc.\n");
    targetLoc.registerNewTargetDetectedCallback(this);
    targetLoc.start();

    lidar.registerInterface(&targetLoc);

    lidar.start(C1Lidar::RPI_SERIAL_DEV);
}

void Window::updateGUI()
{
    cv::Mat leftResized;
    cv::Mat rightResized;

    if (targetLoc.getCurrentLCameraImage().empty())
        return;
    if (targetLoc.getCurrentRCameraImage().empty())
        return;
    if (targetLoc.getCurrentLCameraImage().size !=
        targetLoc.getCurrentRCameraImage().size)
        return;

    cv::resize(targetLoc.getCurrentLCameraImage(), leftResized,
               displayImageSize);
    cv::resize(targetLoc.getCurrentRCameraImage(), rightResized,
               displayImageSize);

    cv::Mat blended;
    cv::addWeighted(leftResized, 0.5, rightResized, 0.5, 0.0, blended);
    const QImage frameD(blended.data, blended.cols, blended.rows, blended.step,
                        QImage::Format_BGR888);
    imageCombined->setPixmap(QPixmap::fromImage(frameD));

    cv::Mat disp8;
    cv::normalize(targetLoc.getCurrentDisparityMap(), disp8, 0, 255,
                  cv::NORM_MINMAX, CV_8U);

    cv::Mat dispBGR;
    cv::cvtColor(disp8, dispBGR, cv::COLOR_GRAY2BGR);

    if (!targetLoc.getQRcodeContour().empty()) {
        std::vector<std::vector<cv::Point>> contours{
            targetLoc.getQRcodeContour()};
        cv::drawContours(dispBGR, contours, -1, {0, 255, 0}, 3);
    }
    const QImage dispImage(dispBGR.data, dispBGR.cols, dispBGR.rows,
                           dispBGR.step, QImage::Format_BGR888);
    imageDisparity->setPixmap(QPixmap::fromImage(dispImage));

    QVector<double> x, y;
    for (const auto &v : targetLoc.getCurrentLidarCoords()) {
        x.append(v.x);
        y.append(v.y);
    }
    lidarPlot->graph(0)->setData(x, y);
    lidarPlot->replot();
}

// properly done with a callback!
void Window::newTargetDetected(const cv::Point2f topViewEgoCoordinate)
{
    QVector<double> xv, yv;
    xv.append(topViewEgoCoordinate.x);
    yv.append(topViewEgoCoordinate.y);
    fprintf(stderr, "Plotting target at [%f,%f].\n", topViewEgoCoordinate.x,
            topViewEgoCoordinate.y);
    lidarPlot->graph(1)->setData(xv, yv);
}

// the rest just with a timer
void Window::timerEvent(QTimerEvent *) { updateGUI(); }
