#include "realWorldTestHeaders.h"
#include "../custom_robot.h"
#include "attentive.h"
const bool DEBUG=false;

/**
* @brief inherits from motor callback in @file custom_robot.h and from closed loop tracker
*/
class CLDriver: public MotorCallback, public ClosedLoop_Tracker{};

int main(int argc, char** argv) {
	C1Lidar lidar;
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputConfigurator configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    CLDriver tracker;
    Task controlGoal(goal, UNDEFINED);
    configurator.register_tracker(&tracker);
    configurator.init(controlGoal);
	OneTaskController rc;
	configurator.register_controller(&rc);
	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface( &tracker);
	lidar.registerInterface(&dataInterface);
	
	lidar.start(RPI_SERIAL_DEV);
	tracker.start();
	do{
    }while(!getchar());
	tracker.stop();
	lidar.stop();
}
	
	
