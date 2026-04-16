#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();
	imageL = new QLabel;
	imageR = new QLabel;
	imageCombined = new QLabel;
	h1Layout = new QHBoxLayout();
	h1Layout->addWidget(imageL);
	h1Layout->addWidget(imageR);
	vLayout->addLayout(h1Layout);
	h2Layout = new QHBoxLayout();
	imageCombined = new QLabel;
	h2Layout->addWidget(imageCombined);
	calibratePushbutton = new QPushButton("Calibrate");
	connect(calibratePushbutton, &QPushButton::clicked, [this]()
			{ triggerCalibration(); });
	h2Layout->addWidget(calibratePushbutton);
	calInfo = new QLabel;
	h2Layout->addWidget(calInfo);
	vLayout->addLayout(h2Layout);
	setLayout(vLayout);
}

void Window::updateImageL(const cv::Mat &mat)
{
	currentL = mat;
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
					   QImage::Format_BGR888);
	imageL->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
	update();
	if (leftImages4Cal.size() < numFrames4Calibration)
	{
		leftImages4Cal.push_back(mat);
	}
	check4Cal();
	blendLR();
}

void Window::updateImageR(const cv::Mat &mat)
{
	currentR = mat;
	const QImage frame(mat.data, mat.cols, mat.rows, mat.step,
					   QImage::Format_BGR888);
	imageR->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
	update();
	if (rightImages4Cal.size() < numFrames4Calibration)
	{
		rightImages4Cal.push_back(mat);
	}
	check4Cal();
	blendLR();
}

void Window::blendLR()
{
	if (currentL.empty())
		return;
	if (currentR.empty())
		return;
	if (currentL.size != currentR.size)
		return;
	cv::Mat blended;
	cv::addWeighted(currentL, 0.5, currentR, 0.5, 0.0, blended);
	const QImage frame(blended.data, blended.cols, blended.rows, blended.step,
					   QImage::Format_BGR888);
	imageCombined->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
}

void Window::triggerCalibration()
{
	if (calibrationTriggered)
		return;
	rightImages4Cal.clear();
	leftImages4Cal.clear();
	calInfo->setText("Collecting Images");
	calibrationTriggered = true;
	calInfo->setText("Calibrating");
}

void Window::check4Cal()
{
	if (!calibrationTriggered)
	{
		// do we have valid calibration info?
		if (stereo.hasValidCalibration)
		{
			calInfo->setText("Calibrated");
		}
		else
		{
			calInfo->setText("No calibration data");
		}
		return;
	}
	// Is there already a calibration going on?
	if (stereo.isCalibrating)
		return;
	// OK so want to calibrate and can do it!
	// Do we have enough images?
	if (rightImages4Cal.size() < numFrames4Calibration)
		return;
	if (leftImages4Cal.size() < numFrames4Calibration)
		return;
	if (calThread.joinable())
	{
		calThread.join();
		return;
	}
	// We have enough images so we can calibrate!
	calThread = std::thread([&]()
							{
		int ret = stereo.calibration(rightImages4Cal,leftImages4Cal);
		if (0 == ret) stereo.saveCal("caldata");
		calibrationTriggered = false; });
}