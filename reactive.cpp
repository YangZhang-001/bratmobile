#include "custom_robot.h"
const bool DEBUG=false;

int main(int argc, char** argv) {
	std::cout<<"Navigating to Target with Brat2"<<std::endl;
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
    ReactiveConfigurator configurator;
	LaserFocus wb;
	configurator.setSimulationStep(.5);
	if (argc>1){
		configurator.init(controlGoal);
		configurator.setSimulationStep(.27);
	}
	configurator.register_worldBuilder(&wb);
	NoPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	Reactive_Controller wc;
	configurator.register_controller(&wc);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface( &tracker);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	lidar.start(rpi_serial_dev);
	motors.start();
	getchar();
	motors.stop();
	lidar.stop();
}
	
	
