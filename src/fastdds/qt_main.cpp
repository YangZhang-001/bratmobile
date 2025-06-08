
#include <QApplication>
#include "window.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	Window window;
	// create the window as a callback for the subscriber
	window.show();

	return app.exec();
}