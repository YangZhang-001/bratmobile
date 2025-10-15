#include "../custom_robot.h"
#include "realWorldTestHeaders.h"


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	LIDAR_In configuratorInterface;
    OpenLooper openLooper;
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputDR configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    Task controlGoal(goal, UNDEFINED);
    configurator.register_tracker(&openLooper);
    configurator.init(controlGoal);
	OpenLoopController rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &openLooper);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&openLooper);
	configurator.start();
	lidar.start();
	motors.start();
	do{
    }while(!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
