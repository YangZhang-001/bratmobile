#include "test_headers.h"

// Disturbance set_target(int& run, b2Transform start){
// 	Disturbance result;
// 	return result;
// }
#undef DEBUG
#define DEBUG true

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputConfigurator configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    ClosedLoop_Tracker tracker;
    Task controlGoal(goal, UNDEFINED);
    configurator.register_tracker(&tracker);
    configurator.init(controlGoal);
	OneTaskController rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	//printf("current vertices size=%i\n", configurator.current_vertices.size());

	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	do{
    }while(!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
