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

	h3Layout = new QHBoxLayout();
	calibratePushbutton = new QPushButton("Calibrate");
	connect(calibratePushbutton, &QPushButton::clicked, [this]()
			{ triggerCalibration(); });
	h3Layout->addWidget(calibratePushbutton);
	calInfo = new QLabel;
	h3Layout->addWidget(calInfo);
	vLayout->addLayout(h3Layout);

	setLayout(vLayout);

	stereo.registerCallback([&](cv::Mat d){currentD = d; refreshDisparity = true;});
}

void Window::updateImageL(const cv::Mat &leftInput)
{
	currentL = Stereo::convertColour2Grey(leftInput);
	const QImage frame(currentL.data, currentL.cols, currentL.rows, currentL.step,
					   QImage::Format_Grayscale8);
	imageL->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
	update();
	if (leftImages4Cal.size() < numFrames4Calibration)
	{
		leftImages4Cal.push_back(currentL);
	}
	check4Cal();
	blendLRandDisplayD();
}

void Window::updateImageR(const cv::Mat &rightInput)
{
	currentR = Stereo::convertColour2Grey(rightInput);
	const QImage frame(currentR.data, currentR.cols, currentR.rows, currentR.step,
					   QImage::Format_Grayscale8);
	imageR->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
	update();
	if (rightImages4Cal.size() < numFrames4Calibration)
	{
		rightImages4Cal.push_back(currentR);
	}
	check4Cal();
	blendLRandDisplayD();
}

void Window::blendLRandDisplayD()
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
					   QImage::Format_Grayscale8);
	imageCombined->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));

	// trigger async calc
	stereo.calcDepthMapAsync(currentL,currentR);

	// however we can put that here in the callback as QT won't like it so we basically
	// "poll" it.
	if (!refreshDisparity) return;
	cv::Mat disp8;
	cv::normalize(currentD, disp8, 0, 255, cv::NORM_MINMAX, CV_8U);
	const QImage dispImage(disp8.data, disp8.cols, disp8.rows, disp8.step,
					   QImage::Format_Grayscale8);
	imageDisparity->setPixmap(QPixmap::fromImage(dispImage.scaledToWidth(displaywidth)));
	refreshDisparity = false;
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