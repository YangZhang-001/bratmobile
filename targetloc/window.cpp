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

void Window::updateImageL(const cv::Mat &leftInput)
{
	cv::resize(leftInput, currentL, imageSize);
	const QImage frame(currentL.data, currentL.cols, currentL.rows, currentL.step,
					   QImage::Format_BGR888);
	imageL->setPixmap(QPixmap::fromImage(frame));
	blendLRandDisplayD();
}

void Window::updateImageR(const cv::Mat &rightInput)
{
	cv::resize(rightInput, currentR, imageSize);
	const QImage frame(currentR.data, currentR.cols, currentR.rows, currentR.step,
					   QImage::Format_BGR888);
	imageR->setPixmap(QPixmap::fromImage(frame));
	blendLRandDisplayD();
}

void Window::blendLRandDisplayD()
{
	if (targetLoc.currentL.empty())
		return;
	if (targetLoc.currentR.empty())
		return;
	if (targetLoc.currentL.size != targetLoc.currentR.size)
		return;
	cv::Mat blended;
	cv::addWeighted(targetLoc.currentL, 0.5, targetLoc.currentR, 0.5, 0.0, blended);
	const QImage frame(blended.data, blended.cols, blended.rows, blended.step,
					   QImage::Format_BGR888);
	imageCombined->setPixmap(QPixmap::fromImage(frame));

	cv::Mat disp8;
	cv::normalize(targetLoc.currentD, disp8, 0, 255, cv::NORM_MINMAX, CV_8U);
	const QImage dispImage(disp8.data, disp8.cols, disp8.rows, disp8.step,
						   QImage::Format_Grayscale8);
	imageDisparity->setPixmap(QPixmap::fromImage(dispImage));
}

void Window::timerEvent(QTimerEvent *event)
{
	blendLRandDisplayD();
}
