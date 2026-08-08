#include "window.h"
#include <QApplication>
#include <QTimer>
#include <csignal>

namespace {

	// terminal stop requst flag
	volatile std::sig_atomic_t quitRequested = 0;

	// signal handler for SIGINT and SIGTERM
	void handleSignal(int)
	{
		quitRequested = 1;
	}
}

// Main program
int main(int argc, char *argv[])
{
	QApplication app(argc, argv);

	// Catch terminal stop signals.
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    // Qt-side signal polling.
    QTimer signalTimer;
    QObject::connect(&signalTimer, &QTimer::timeout, [&app]() {
        // Leave through Qt event loop.
        if (quitRequested)
            app.quit();
    });
    signalTimer.start(100);
	
	// create the window
	Window window;
	window.setWindowState(Qt::WindowMaximized);
	window.show();

	const int r = app.exec();

	return r;
}
