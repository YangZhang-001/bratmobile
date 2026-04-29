#include "../custom_robot.h"
#include "realWorldTestHeaders.h"
const bool DEBUG=false;


int main(int argc, char** argv) {
	C1Lidar lidar;
    OpenLooper tracker;
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputDR configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    Task controlGoal(goal, UNDEFINED);
    configurator.register_tracker(&tracker);
    configurator.init(controlGoal);
	OpenLoopController rc;
	configurator.register_controller(&rc);
	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface(&tracker);
	lidar.registerInterface(&dataInterface);
	
	lidar.start(C1Lidar::RPI_SERIAL_DEV);
	tracker.start();
	do{
    }while(!getchar());
	tracker.stop();
	lidar.stop();
}
	
	
