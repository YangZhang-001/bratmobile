#include "custom_robot.h"

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
    FocusedConfigurator configurator;
	configurator.init(controlGoal);
	HorizonStarPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	Wise_Controller wc;
	configurator.register_controller(&wc);
	Logger logger( "rt-update-targetless", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface( &tracker);
	MotorCallback cb(&tracker);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	printf("all registered\n");
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	lidar.stop();
	logger.~Logger();
}
	
	
