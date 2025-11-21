#include "custom_robot.h"

int main(int argc, char** argv) {
	std::cout<<"Navigating to Target with Brat2"<<std::endl;
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
	//Motor_Out controlInterface;
    ReactiveConfigurator configurator;
	if (argc>1){
		configurator.init(controlGoal);
		configurator.setSimulationStep(.27);
	}
	else{
		configurator.setSimulationStep(.5);
	}
	LaserFocus wb;
	configurator.register_worldBuilder(&wb);
	NoPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	Reactive_Controller Controller wc;
	configurator.register_controller(&wc);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &tracker);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	configurator.stop();
	lidar.stop();
}
	
	
