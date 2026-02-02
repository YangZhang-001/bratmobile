#include "custom_robot.h"
#include "gui.h"


class OpenLooperGUI: public OLTrackerGUI, public MotorCallback, public Motor_Out{
    int motorStep=0;
    public:
    OpenLooper():MotorCallback(this){}

    void on_new_task(const Task &task, const Task & goal){
        motorStep=task.getMotorStep();
        std::cout<<"motorStep="<<motorStep<<std::endl;
        deltaTransform=b2Transform_zero;
    }

    bool hasTaskEnded(Task & t)override{
        return motorStep<=0;
        
    }

    void step(AlphaBot& motors)override{
        if (L!=0 && R!=0){
            motorStep--;
            std::cout<<"one down"<<std::endl;
            std::cout<<"motorStep="<<motorStep<<std::endl;
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
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
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
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &tracker);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&tracker);
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	configurator.stop();
	lidar.stop();
	logger.~Logger();
}
	
	
