
#include <QApplication>
#include "window.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	// create the window
	Window window;
	window.show();

	// execute the application
	return app.exec();
}