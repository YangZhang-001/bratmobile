#include "custom_robot.h"

class NoGoal:public GoalChanger{

	Task change_goal(const Task & task){
		return Task();
	}

};

class CLTracker: public ClosedLoop_Tracker, public MotorCallback, public Motor_Out{
	public:
	CLTracker():MotorCallback(this){}
};

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
	//Motor_Out controlInterface;
    DiscreteConfigurator configurator(controlGoal);
	NoGoal goalChanger;
	configurator.register_goalChanger(&goalChanger);
	HorizonStarPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	OpenLoopController wc;
	configurator.register_controller(&wc);
	Logger logger( "rt-update-targetless", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &tracker);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	printf("all registered\n");
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	configurator.stop();
	lidar.stop();
	logger.~Logger();
}
	
	

	

	
