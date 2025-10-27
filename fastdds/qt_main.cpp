#include <QApplication>
#include <QMetaType>
#include "window.h"

Q_DECLARE_METATYPE(UnpackedObject)

int main(int argc, char *argv[])
{
	qRegisterMetaType<UnpackedObject>("UnpackedObject");
	QApplication app(argc, argv);

	RobotSubscriber DiSub, goalSub, attentionSub;
	RobotReaderListener DiListener, goalListener, attentionListener;
	DiSub.setTopic(Di_topic);
	goalSub.setTopic(Goal_topic);
	attentionSub.setTopic(attention_topic);
	DiSub.registerListener(&DiListener);
	goalSub.registerListener(&goalListener);
	attentionSub.registerListener(&attentionListener);

	if(!DiSub.init()){std::cerr << "Could not init the Di subscriber." << std::endl;return -1;}        
	if(!goalSub.init()){std::cerr << "Could not init the goal subscriber." << std::endl;return -1;}        
	if(!attentionSub.init()){std::cerr << "Could not init the attention subscriber." << std::endl;return -1;}        

	Window window;

	QObject::connect(&DiListener, &RobotReaderListener::newObject, &window, &Window::setDi);
	QObject::connect(&goalListener, &RobotReaderListener::newObject, &window, &Window::setGoal);
	QObject::connect(&attentionListener, &RobotReaderListener::newObject, &window, &Window::setAttention);

	window.show();

	//window.start();

	return app.exec();
}