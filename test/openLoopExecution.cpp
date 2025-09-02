#include "../custom_robot.h"
#include "realWorldTestHeaders.h"
/**
 * Tracks and executes tasks using deadreckoning
 */
class OpenLooper: public DeadReckoner, public MotorCallback, public Motor_Out{
    int motorStep=0;
    public:
    OpenLooper():MotorCallback(this){
        
    }

    void on_new_task(const Task &task, const Task & goal){
        motorStep=task.getMotorStep();
        std::cout<<"motorStep="<<motorStep<<std::endl;
        deltaTransform=b2Transform_zero;
    }

    bool hasTaskEnded(Task & t)override{
        return motorStep<=0;
        
    }

    void step(AlphaBot& motors)override{
        float R_adjust=R*1.3;
        float L_adjust=L*1.3;
        motors.setRightWheelSpeed(R_adjust); //temporary fix because motors on despacito are the wrong way around
        motors.setLeftWheelSpeed(L_adjust);
        if (L!=0 && R!=0){
            motorStep--;
            std::cout<<"one down"<<std::endl;
            std::cout<<"motorStep="<<motorStep<<std::endl;
        }
        if (motorStep==0){
            L=0;
            R=0;
            motors.setLeftWheelSpeed(0);
            motors.setRightWheelSpeed(0);
        }
    }
};
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
	
	
