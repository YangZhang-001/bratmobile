#include "../custom_robot.h"
#include "realWorldTestHeaders.h"


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
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
	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface(&openLooper);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&openLooper);
	lidar.start();
	motors.start();
	do{
    }while(!getchar());
	motors.stop();
	lidar.stop();
}
	
	
