#include "custom_robot.h"

Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	if (run%2!=0){
		result= Disturbance(PURSUE, b2Vec2(1.0f, 0.0f), 0.0f);
	}
	else{
		result= Disturbance(PURSUE, b2Vec2(-1.0f, 0.0f), M_PI);
	}
	return result;
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    AttentiveConfigurator configurator(controlGoal);
	HorizonStarPlanner planner;
	DeadReckoner tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	Wise_Controller wc;
	configurator.register_controller(&wc);
	Logger logger( "rt-update-targetless", "/tmp");
	if (argc>1){
		#define DEBUG atoi(argv[1])
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	printf("all registered\n");
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	configurator.stop();
	lidar.stop();
	~logger;
}
	
	
