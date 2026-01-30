#include <QApplication>
#include <QMetaType>
#include "window.h"
#include "CppTimer.h"

Q_DECLARE_METATYPE(UnpackedObject)

int main(int argc, char *argv[])
{
	qRegisterMetaType<UnpackedObject>("UnpackedObject");
	QApplication app(argc, argv);

	ObjectPackageSubscriber DiSub, goalSub, attentionSub;
	RobotReaderListener DiListener, goalListener, attentionListener;
	DiSub.registerListener(&DiListener);
	goalSub.registerListener(&goalListener);
	attentionSub.registerListener(&attentionListener);	
	if(!DiSub.init(Di_topic)){std::cerr << "Could not init the Di subscriber." << std::endl;return -1;}        
	if(!goalSub.init(Goal_topic)){std::cerr << "Could not init the goal subscriber." << std::endl;return -1;}        
	if(!attentionSub.init(attention_topic)){std::cerr << "Could not init the attention subscriber." << std::endl;return -1;}        
	// DiSub.setTopic(Di_topic);
	// goalSub.setTopic(Goal_topic);
	// attentionSub.setTopic(attention_topic);



	Window window;

	QObject::connect(&DiListener, &RobotReaderListener::newObject, &window, &Window::setDi);
	QObject::connect(&goalListener, &RobotReaderListener::newObject, &window, &Window::setGoal);
	QObject::connect(&attentionListener, &RobotReaderListener::newObject, &window, &Window::setAttention);

	window.show();

	//window.start();

	return app.exec();
}