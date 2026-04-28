#include "custom_robot.h"
const bool DEBUG=false;

int main(int argc, char** argv) {
	std::cout<<"Navigating to Target with Brat2"<<std::endl;
	C1Lidar lidar;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
    FocusedConfigurator configurator;
	configurator.init(controlGoal);
	HorizonStarPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	OpenLoopController wc;
	configurator.register_controller(&wc);
	Logger logger( "brat3-target", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface(&tracker);
	lidar.registerInterface(&dataInterface);
	
	lidar.start(RPI_SERIAL_DEV);
	tracker.start();
	getchar();
	tracker.stop();
	lidar.stop();
	logger.~Logger();
}
	
	
