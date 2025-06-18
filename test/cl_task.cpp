#include "custom_robot.h"

#undef PLANNING
#define PLANNING false


class AffordanceSetter{
	public:
	AffordanceIndex affordance=NONE;
	AffordanceSetter()=default;
	AffordanceSetter(AffordanceIndex i):affordance(i){}

}as;

class TaskSetter{
	public:
	Direction d=DEFAULT;
	bool topDown=false;

	TaskSetter()=default;

	TaskSetter(Direction _d):d(_d){
		topDown=true;
	}
	
}ts;


Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	return result;
}
#undef DEBUG
#define DEBUG true

int main(int argc, char** argv) {
	#undef PLANNING
	#define PLANNING false
	printf("PLANNING =%i\n", PLANNING);
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    Configurator configurator(controlGoal);
	ClosedLoop_Tracker tracker;
	configurator.register_tracker(&tracker);
	Reactive_Controller rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	printf("current vertices size=%i\n", configurator.current_vertices.size());
	as=AffordanceSetter(AffordanceIndex(atoi(argv[1])));
	if (argc>3){
		ts =TaskSetter(Direction(atoi(argv[3])));
	}
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
