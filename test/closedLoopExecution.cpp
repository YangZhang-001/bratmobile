#include "realWorldTestHeaders.h"
#include "../custom_robot.h"
#include "attentive.h"
const bool DEBUG=false;

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	MotorCallback controlInterface;
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
	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface( &controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&controlInterface);
	lidar.start();
	motors.start();
	do{
    }while(!getchar());
	motors.stop();
	lidar.stop();
}
	
	
