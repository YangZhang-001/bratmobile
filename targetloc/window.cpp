#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();

	h1Layout = new QHBoxLayout();
	imageL = new QLabel;
	imageR = new QLabel;
	h1Layout->addWidget(imageL);
	h1Layout->addWidget(imageR);
	vLayout->addLayout(h1Layout);

	h2Layout = new QHBoxLayout();
	imageCombined = new QLabel;
	h2Layout->addWidget(imageCombined);
	imageDisparity = new QLabel;
	h2Layout->addWidget(imageDisparity);
	vLayout->addLayout(h2Layout);

	setLayout(vLayout);

	startTimer(std::chrono::milliseconds{100});

	targetLoc.start();
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

	const QImage frameL(leftResized.data, leftResized.cols, leftResized.rows, leftResized.step,
					   QImage::Format_BGR888);
	imageL->setPixmap(QPixmap::fromImage(frameL));

	const QImage frameR(rightResized.data, rightResized.cols, rightResized.rows, rightResized.step,
					   QImage::Format_BGR888);
	imageR->setPixmap(QPixmap::fromImage(frameR));
}

void Window::timerEvent(QTimerEvent*)
{
	updateGUI();
}
