#include "custom_robot.h"
const bool DEBUG=false;

class NoGoal:public GoalChanger{

	Task change_goal(const Task & task){
		return Task();
	}

};

// class CLTracker: public ClosedLoop_Tracker, public MotorCallback, public Motor_Out{
// 	public:
// 	CLTracker():MotorCallback(this){}
// };

int main(int argc, char** argv) {
	C1Lidar lidar;
	//Motor_Out controlInterface;
    FocusedConfigurator configurator;
	LaserFocus wb;
	configurator.register_worldBuilder(&wb);
	NoGoal goalChanger;
	configurator.register_goalChanger(&goalChanger);
	HorizonStarPlanner planner;
	OpenLooper tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	OpenLoopController wc;
	configurator.register_controller(&wc);
	Logger logger( "brat4-targetless", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface(&tracker);
	lidar.registerInterface(&dataInterface);

	printf("all registered\n");
	lidar.start(RPI_SERIAL_DEV);
	tracker.start();
	getchar();
	tracker.stop();
	lidar.stop();
	logger.~Logger();
}
	
	

	

	
