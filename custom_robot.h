#ifndef CUSTOM_INTERFACES
#define CUSTOM_INTERFACES
#include "a1lidarrpi.h"
#include "alphabot.h"
#include "attentive.h"
//#include "Iir.h"
//#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#define _USE_MATH_DEFINES

/**
 * * * * DEFINITION OF DATA INTERFACES FOR ROBOT SENSORS/MOTORS
 * 				+ SOME DEBUGGING HELPER FUNCTIONS
 *
 */


class LidarInterface : public A1Lidar::DataInterface{
LIDAR_In * ci=NULL;
public:
    int mapCount =0;

    LidarInterface(LIDAR_In * _ci): ci(_ci){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (ci == NULL){
			std::cerr<<"null pointer to ci"<<std::endl;
			return;
		}
    	ci->setReady(false);
		ci->data2fp.clear();
		mapCount++;
		Pointf p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		if (ci->debugOn){
			f=fopen(name, "w");
		}
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x2 = round(data.x*100)/100; //resolution adjus
				float y2 = round(data.y*100)/100;
				p2f=Pointf(x2, y2);
				ci->data2fp.insert(p2f);
				if (ci->debugOn){
					fprintf(f, "%.2f\t%.2f\n", p2f.x, p2f.y);
				}
            }
		}
		if (ci->debugOn){
		fclose(f);
		}
		ci->setReady(1);
		ci->iteration++;

	}


};

class MotorCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
protected:
	Motor_Out * mio;
public:

MotorCallback(Motor_Out *_mio): mio(_mio){}
virtual void step( AlphaBot &motors){
	if (mio==NULL){
		std::cout<<("no motor out interface");
	}
    motors.setRightWheelSpeed(mio->get_R()); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(mio->get_L()*1.15);
	printf(",R=%f\tL=%f\n",mio->get_R(), mio->get_L());
}
};

/**
	@brief executes plans in open loop, doesn't check plan or recycle
*/
class TentativeConfigurator: public AttentiveConfigurator{

	void explore_plan(b2World & world){
		if (iteration<=1){
			AttentiveConfigurator::explore_plan(world);
		}
	}
	public:

	TentativeConfigurator(): AttentiveConfigurator(){}

	TentativeConfigurator(const Task & goal): AttentiveConfigurator(goal){}
};

/**
 * @brief Tracks and executes tasks using deadreckoning. child of DeadReckoner, MotorCallback and Motor_Out
 * Tracks execution using dead reckoning, i.e. without LiDAR input. Executes task for a given number of steps
 * determined upon task simulation. Automatically registers Motor_Out interface to the
 * MotorCallback, which tracks the number of steps and stops the robot when the task ends.
 */
class OpenLooper: public DeadReckoner, public MotorCallback, public Motor_Out{
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
        motors.setRightWheelSpeed(R*1.20);
		
    }
};


#endif


