#include "custom_robot.h"

int main(int argc, char** argv) {
	std::cout<<"Navigating to Target with Brat2"<<std::endl;
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	//Motor_Out controlInterface;
    DiscreteConfigurator configurator;
	configurator.init(controlGoal);
	LaserFocus wb;
	configurator.register_worldBuilder(&wb);
	HorizonStarPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	OpenLoopController wc;
	configurator.register_controller(&wc);
	Logger logger( "brat2-target", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface(&tracker);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	lidar.stop();
	logger.~Logger();
}
	
	
