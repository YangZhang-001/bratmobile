#include "window.h"

Window::Window()
{
	vLayout = new QVBoxLayout();
	imageL = new QLabel;
	vLayout->addWidget(imageL);
	targetInfo = new QLabel;
	vLayout->addWidget(targetInfo);
	setLayout(vLayout);
}

void Window::updateImageL(const cv::Mat &leftInput)
{
    cv::Mat greyImage;
    cv::cvtColor(leftInput, greyImage, cv::COLOR_BGR2GRAY);
	const QImage frame(greyImage.data, greyImage.cols, greyImage.rows, greyImage.step,
					   QImage::Format_Grayscale8);
	imageL->setPixmap(QPixmap::fromImage(frame.scaledToWidth(displaywidth)));
	update();
	targetDet.detectAsync(leftInput);
	std::string text = "Coord: ";
	text = text + std::to_string(qrCoord.x) + "," + std::to_string(qrCoord.y);
	targetInfo->setText(text.c_str());
}
