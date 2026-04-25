#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();

	series = new QScatterSeries();
	series->setMarkerSize(5.0);

	QVector<QPointF> lidarPoints = {
		{100.0, 200.0},
		{200.0, 350},
		{300, 102}};

	for (const QPointF &p : lidarPoints)
		series->append(p);

	chart = new QChart();
	chart->addSeries(series);
	chart->createDefaultAxes();
	chart->setTitle("LiDAR XY Plot");

	chartView = new QChartView(chart);
	chartView->setRenderHint(QPainter::Antialiasing);

	vLayout->addWidget(chartView);

	h2Layout = new QHBoxLayout();
	imageCombined = new QLabel;
	h2Layout->addWidget(imageCombined);
	imageDisparity = new QLabel;
	h2Layout->addWidget(imageDisparity);
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

	series->clear();
	for(auto& v:targetLoc.getCurrentLidarCoords()) {
		series->append({v.x,v.y});
	}
}

void Window::timerEvent(QTimerEvent *)
{
	updateGUI();
}
