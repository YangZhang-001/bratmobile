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

const bool DEBUG=false;

/**
 * * * * DEFINITION OF DATA INTERFACES FOR ROBOT SENSORS/MOTORS
 * 				+ SOME DEBUGGING HELPER FUNCTIONS
 *
 */


class LidarInterface : public A1Lidar::DataInterface{
Configurator * configurator=NULL;
bool debugOn=false; 
int mapCount =0;

public:

	void setDebug(bool on){
		debugOn=on;
	}

    LidarInterface(Configurator * _c): configurator(_c){}

	void newScanAvail(float, A1LidarData (&data)[A1Lidar::nDistance]){ //uncomment sections to write x and y to files
		if (configurator == NULL){
			std::cerr<<"girl where's the configurator"<<std::endl;
			return;
		}
		mapCount++;
		Pointf p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		printf("%s\n", name);
		configurator->clearData();
		if (debugOn){
			f=fopen(name, "w");
		}
		for (A1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x = round(data.x*100)/100; //resolution adjus
				float y = round(data.y*100)/100;
				configurator->insertCoordinate(x, y);
				if (debugOn){
					fprintf(f, "%.2f\t%.2f\n",x , y);
				}
            }
		}
		if (debugOn){
		fclose(f);
		}
		configurator->newScanEvent();

	}


};

class MotorCallback :public AlphaBot::StepCallback, public MotorInterface { //every 100ms the callback updates the plan
public:

virtual void step( AlphaBot &motors){
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L*1.15);
	printf(",R=%f\tL=%f\n",R, L);
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

	TentativeConfigurator(const Task & goal): FocusedConfigurator(goal){}
};

/**
 * @brief Tracks and executes tasks using deadreckoning. child of DeadReckoner, MotorCallback and Motor_Out
 * Tracks execution using dead reckoning, i.e. without LiDAR input. Executes task for a given number of steps
 * determined upon task simulation. Automatically registers Motor_Out interface to the
 * MotorCallback, which tracks the number of steps and stops the robot when the task ends.
 */
class OpenLooper: public DeadReckoner, public MotorCallback{
    int motorStep=0;
    public:

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


#endif


