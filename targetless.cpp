
#include "custom_robot.h"

Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	if (run%2==0){
		result=Disturbance(PURSUE, start.p, start.q.GetAngle());
		run++;
	}
	return result;
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    AttentiveConfigurator configurator(controlGoal);
	DeadReckoner tracker;
	configurator.register_tracker(&tracker);	
	Wise_Controller wc;
	configurator.register_controller(&wc);
	char name[60];
	dump_benchmarks( "rt-update-targetless", "/tmp");
	if (argc>1){
		#define DEBUG atoi(argv[1])
		//configuratorInterface.debugOn = atoi(argv[1]);
	}	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();
	configurator.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
