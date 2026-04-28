#include "../custom_robot.h"
#include "gui.h"

const bool DEBUG=false;

class OpenLooperGUI: public OLTrackerGUI, public MotorCallback{
    int motorStep=0;
    public:

    void on_new_task(const Task &task, const Task & goal){
        motorStep=task.getMotorStep();
        deltaTransform=b2Transform_zero;
    }

    bool hasTaskEnded(Task & t)override{
        return motorStep<=0;
        
    }

    void step(AlphaBot& motors)override{
        if (L!=0 && R!=0){
            motorStep--;
		}
        if (motorStep==0){
            L=0;
            R=0;
        }
		motors.setLeftWheelSpeed(L*1.18);
        motors.setRightWheelSpeed(R*1.18);
    }
};

int main(int argc, char** argv) {
	std::cout<<"Navigating to Target with Brat2"<<std::endl;
	C1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	//Motor_Out controlInterface;
    FocusedConfigurator configurator;
	configurator.init(controlGoal);
	LaserFocus wb;
	configurator.register_worldBuilder(&wb);
	HorizonStarPlanner planner;
	OpenLooperGUI tracker;
	configurator.register_planner(&planner);
	configurator.register_tracker(&tracker);
	OpenLoopController wc;
	configurator.register_controller(&wc);
	Logger logger( "brat4-target", "/tmp");
	configurator.register_logger(&logger);
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface( &tracker);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	lidar.start(RPI_SERIAL_DEV);
	motors.start();
	getchar();
	motors.stop();
	lidar.stop();
	logger.~Logger();
}
	
	
