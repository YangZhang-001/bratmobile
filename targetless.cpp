#include "custom_robot.h"
const bool DEBUG=false;

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
	C1Lidar lidar;
	LaserFocus wb;
	//Motor_Out controlInterface;
    FocusedConfigurator configurator;
	NoGoal goalChanger;
	configurator.register_goalChanger(&goalChanger);
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
	
	printf("all registered\n");
	configurator.start();
	lidar.start(C1Lidar::RPI_SERIAL_DEV);
	tracker.start();
	getchar();
	tracker.stop();
	configurator.stop();
	lidar.stop();
	logger.~Logger();
}
	
	

	

	
