#ifndef CUSTOM_INTERFACES
#define CUSTOM_INTERFACES
#include "c1lidarrpi.h"
#include "zetabot.h"
#include "attentive.h"
//#include "Iir.h"
//#include "CppTimer.h"
#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#define _USE_MATH_DEFINES


/**
 * * * * DEFINITION OF DATA INTERFACES FOR ROBOT SENSORS/MOTORS
 * 				+ SOME DEBUGGING HELPER FUNCTIONS
 *
 */


class LidarInterface : public C1Lidar::DataInterface{
Configurator * configurator=NULL;

// targetInterface != NULL: target acquisition
// both pointers are NULL: transition; configurator != NULL: navigation
C1Lidar::DataInterface * targetInterface=NULL;
std::mutex routeMutex;

int mapCount =0;

public:

    LidarInterface(Configurator * _c): configurator(_c){}

    /**
     * @brief startTargetAcquisition sets the targetInterface to the given pointer and clears the configurator.
     * @param _target pointer to the target data interface.
     */
    void startTargetAcquisition(C1Lidar::DataInterface * _target){
    std::lock_guard<std::mutex> guard(routeMutex);
    configurator=NULL;
    targetInterface=_target;
    }

    void beginTransition(){
        std::lock_guard<std::mutex> guard(routeMutex);
        targetInterface=NULL;
        configurator=NULL;
    }
 
    void startNavigation(Configurator * _c){
        std::lock_guard<std::mutex> guard(routeMutex);
        targetInterface=NULL;
        configurator=_c;
    }

	void newScanAvail(C1LidarData (&data)[C1Lidar::nDistance]) override{ //uncomment sections to write x and y to files
		std::lock_guard<std::mutex> guard(routeMutex);
        
        if (targetInterface != NULL){
			targetInterface->newScanAvail(data);
			return;
		}

        if (configurator == NULL) {
            return;
        }
		mapCount++;
		Pointf p2f;
		FILE *f;
		char name[256];
		sprintf(name,"/tmp/map%04i.dat", mapCount);
		//printf("%s\n", name);
		configurator->clearData();
		if (DEBUG){
			f=fopen(name, "w");
		}
		for (C1LidarData &data:data){
			if (data.valid&& data.r <LIDAR_RANGE){
				float x = round(data.x*100)/100; //resolution adjus
				float y = round(data.y*100)/100;
				configurator->insertCoordinate(x, y);
				if (DEBUG){
					fprintf(f, "%.2f\t%.2f\n",x , y);
				}
            }
		}
		if (DEBUG){
			fclose(f);
		}
		configurator->newScanEvent();
	}

};

class MotorCallback :public ZetaBot, public MotorInterface { //every 100ms the callback updates the plan
public:

void getData(const Task::Action &a)override{
	MotorInterface::getData(a);
	std::cout<<"New data received!"<<std::endl;
	otherStuff();
    setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    setLeftWheelSpeed(L);
}

/**
* A function to implement any other procedure before wheel speeds are changed
*/
virtual void otherStuff(){
	std::cout<<"Base class"<<std::endl;
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
        deltaTransform=b2Transform_zero;
    }

    bool hasTaskEnded(Task & t)override{
		//std::cout<<"Open loop checking task has ended ="<<int(motorStep<=0) <<std::endl;
        return motorStep<=0;
    }

	void otherStuff()override{
		std::cout<<"Open looping!"<<std::endl;
        // if (L!=0 && R!=0){
        //     motorStep--;
        // }
        if (motorStep==0){
            L=0;
            R=0;
        }
		// std::cout<<"Motor step "<<motorStep<<std::endl;
	}

	void on_new_reading(const Task &task, const Task & goal){
        if (L!=0 && R!=0){
            motorStep--;
        }
        if (motorStep==0){
            L=0;
            R=0;
        }
		std::cout<<"Motor step "<<motorStep<<std::endl;
		printf("R=%f\tL=%f\n",R, L);

    }

};


#endif


