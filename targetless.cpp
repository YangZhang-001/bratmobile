#include "custom_robot.h"

class NoGoal:public GoalChanger{

	Task change_goal(const Task & task){
		return Task();
	}

};

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
	HorizonStarPlanner planner;
    TentativeConfigurator configurator;
	ClosedLoop_Tracker tracker;
	NoGoal goalChanger;
	configurator.register_tracker(&tracker);	
	configurator.register_planner(&planner);
	configurator.register_goalChanger(&goalChanger);
	Wise_Controller wc;
	configurator.register_controller(&wc);
	char name[60];
	Logger logger( "rt-update-targetless", "/tmp");
	configurator.register_logger(&logger);
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
	logger.~Logger();
}
	
	
