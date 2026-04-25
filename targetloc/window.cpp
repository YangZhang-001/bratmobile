#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();
	h1Layout = new QHBoxLayout();

	customPlot = new QCustomPlot;
	customPlot->addGraph();
	customPlot->graph(0)->setLineStyle(QCPGraph::lsNone);
	customPlot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);

	lidarData.reset(new QCPDataContainer<QCPGraphData>);
	for (int i = 0; i < 9000; i++)
	{
		QCPGraphData data(0, 0);
		lidarData->add(data);
	}
	customPlot->graph()->setData(lidarData);

	customPlot->xAxis->setRange(-5, 5);
	customPlot->yAxis->setRange(-5, 5);

	h1Layout->addWidget(customPlot);

	imageDisparity = new QLabel;
	h1Layout->addWidget(imageDisparity);

	vLayout->addLayout(h1Layout);

	h2Layout = new QHBoxLayout();
	imageCombined = new QLabel;
	h2Layout->addWidget(imageCombined);
	vLayout->addLayout(h2Layout);

	setLayout(vLayout);

	fprintf(stderr, "Starting screen update timer.\n");
	startTimer(std::chrono::milliseconds{100});

	fprintf(stderr, "Starting Targetloc.\n");
	targetLoc.start();

	lidar.registerInterface(&targetLoc);

	lidar.start(RPI_SERIAL_DEV);
}

void Window::updateGUI()
{
	cv::Mat leftResized;
	cv::Mat rightResized;

	if (targetLoc.getCurrentLCameraImage().empty())
		return;
	if (targetLoc.getCurrentRCameraImage().empty())
		return;
	if (targetLoc.getCurrentLCameraImage().size != targetLoc.getCurrentRCameraImage().size)
		return;

	cv::resize(targetLoc.getCurrentLCameraImage(), leftResized, displayImageSize);
	cv::resize(targetLoc.getCurrentRCameraImage(), rightResized, displayImageSize);

	cv::Mat blended;
	cv::addWeighted(leftResized, 0.5, rightResized, 0.5, 0.0, blended);
	const QImage frameD(blended.data, blended.cols, blended.rows, blended.step,
						QImage::Format_BGR888);
	imageCombined->setPixmap(QPixmap::fromImage(frameD));

	cv::Mat disp8;
	cv::normalize(targetLoc.getCurrentDisparityMap(), disp8, 0, 255, cv::NORM_MINMAX, CV_8U);
	const QImage dispImage(disp8.data, disp8.cols, disp8.rows, disp8.step,
						   QImage::Format_Grayscale8);
	imageDisparity->setPixmap(QPixmap::fromImage(dispImage));


	lidarData.clear();
	for (const auto &v : targetLoc.getCurrentLidarCoords())
	{
	    QCPGraphData data(v.x,v.y);
	    lidarData->add(data);
	}
	customPlot->replot();
}

void Window::timerEvent(QTimerEvent *)
{
	updateGUI();
}
