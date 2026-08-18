#ifndef CUSTOM_INTERFACES
#define CUSTOM_INTERFACES
#include "c1lidarrpi.h"

#ifdef BRAT_USE_ROCK5_WHEELEDDRIVE
#include "wheeleddrive/Driving.h"
#include <exception>
#else
#include "zetabot.h"
#endif

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

/**
 * @brief connects navigation motor commands to the selected robot backend.
 *
 * the Raspberry Pi path keeps the existing ZetaBot interface, while the
 * Rock 5 path uses Driving. Rock 5 dry-run mode prints navigation commands
 * without accessing PWM, so command direction and sequencing can be checked
 * before powered navigation and full TargetLoc shutdown are validated.
 */

#ifdef BRAT_USE_ROCK5_WHEELEDDRIVE
class MotorCallback : public MotorInterface {
#else
class MotorCallback : public ZetaBot, public MotorInterface {
#endif

public:

#ifdef BRAT_USE_ROCK5_WHEELEDDRIVE
bool start()
{
#ifdef BRAT_ROCK5_MOTOR_DRY_RUN
    std::cout << "Rock 5 navigation motor dry-run enabled.\n";
    return true;
#else
    try {
        driving.start();
        std::cout << "Rock 5 wheeleddrive backend started.\n";
        return true;
    } catch (const std::exception &e) {
        std::cerr << "Could not start Rock 5 wheeleddrive backend: "
                  << e.what() << '\n';
        return false;
    }
#endif
}

void stop()
{
#ifndef BRAT_ROCK5_MOTOR_DRY_RUN
    driving.stop();
#endif
}
#endif

void getData(const Task::Action &a) override
{
    MotorInterface::getData(a);
    std::cout << "New data received!" << std::endl;
    otherStuff();

#ifdef BRAT_USE_ROCK5_WHEELEDDRIVE

#ifdef BRAT_ROCK5_MOTOR_DRY_RUN
    if (L != lastCommandLeft || R != lastCommandRight) {
        std::cout
            << "Rock 5 motor dry-run: command not sent, L="
            << L << ", R=" << R << '\n';

        lastCommandLeft = L;
        lastCommandRight = R;
    }
#else
    const int result = driving.setMotorSpeeds(L, R);

    if (result < 0) {
        std::cerr << "Could not set Rock 5 motor speeds.\n";
    }
#endif

#else
    setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    setLeftWheelSpeed(L);
#endif
}

/**
* A function to implement any other procedure before wheel speeds are changed
*/
virtual void otherStuff()
{
    std::cout << "Base class" << std::endl;
}

#ifdef BRAT_USE_ROCK5_WHEELEDDRIVE
private:

    Driving driving;

#ifdef BRAT_ROCK5_MOTOR_DRY_RUN
    float lastCommandLeft = 2.0F;
    float lastCommandRight = 2.0F;
#endif

#endif
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
class OpenLooper: public DeadReckoner, public MotorCallback, public TurnStepSource{
    int motorStep=0;


    // +LEFT executed step, -RIGHT executed step.
    int turnStepBalance=0;

    public:

    void on_new_task(const Task &task, const Task & goal){
        motorStep=task.getMotorStep();
        deltaTransform=b2Transform_zero;
    }

    bool hasTaskEnded(Task & t)override{
		//std::cout<<"Open loop checking task has ended ="<<int(motorStep<=0) <<std::endl;
        return motorStep<=0;
    }

    // expose only the accumulated executed turn steps.
    int getTurnStepBalance() const override{
        return turnStepBalance;
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

            // count only executed turning; straight motion has zero omega.
            const float omega=task.getAction().getOmega();

            if (omega>0){
                // LEFT contributes a positive step.
                turnStepBalance++;
            }
            else if (omega<0){
                // RIGHT contributes a negative step.
                turnStepBalance--;
            }
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


