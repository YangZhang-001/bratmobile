
#include <QApplication>
#include "window.h"

int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	Window window;
	// create the window as a callback for the subscriber
	DDSQtListener listener;
	window.registerListener(&listener);
	window.show();
    if(!window.init())
    {
	std::cerr << "Could not init the subscriber." << std::endl;
	return -1;
    }
	// do{

	// }while(!getchar());
	// execute the application
	return app.exec();;
}