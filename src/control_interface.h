#ifndef CONTROL_IF_H
#define CONTROL_IF_H

#include "worldbuilder.h"
#include "task.h"

class Configurator; 

/**
* Input/Output interface for Configurator
*/
// class IOInterface{
// 	protected:
// 	bool ready=false;
// 	public:
// 	IOInterface()=default;

// 	bool isReady(){
// 		return ready;
// 	}

// 	void setReady(bool b){
// 		ready=b;
// 	}

// };

/**
* Receives LIDAR data
*/
// class LIDAR_In:public IOInterface{ 
// public:
// 	bool debugOn=0;
// 	int iteration=0;
// 	bool stop=0;
// 	CoordinateContainer data2fp;
// };

/**
* Output from Configurator to Motors
*/
class MotorInterface { 
	protected:
	float L=0, R=0, L_gain=1.0f, R_gain=1.0f, Kp=0.45, Ki=0.25, Kd=0.2; //from empirical, Kp should be 1.2
	float prev_error=0;
	float integral=0;
    public:

	MotorInterface()=default;

	MotorInterface(float kp, float ki, float kd):Kp(kp), Ki(ki), Kd(kd){}

	void getData(const Task::Action &a){
		L=a.getLWheelSpeed();
		R=a.getRWheelSpeed();
	}

	float get_L(){
		float f=L*L_gain;
		return f;
	}

	float get_R(){
		float f=R*R_gain;
		return f;
	}

	/**
	 * @brief Adjusts L/R wheel gain, implements a PID controller with option to turn it into cascaded controller
	 * 
	 * @param angle_D desired angle from disturbance
	 * @param observed observed transform
	 * @param y_D desired distance from disturbance on the y axis (pointer so optional) 
	 */
	void adjust_gain( float angle_D, b2Transform observed, float * y_D=NULL);

	/**
	 * @brief Implements a PID controller
	 * 
	 * @param e 
	 */
	void PID(float e);

	/**
	 * @brief Implements a P controller in the outer loop of the cascade controller
	 * 
	 * max dl/dt and dr/dt = 2.0 (going from -1->1 and viceversa)
	 * max_corrective angle= ((max_dl-max_dr)/speed)*LIDAR_SAMPLING_RATE=(((-2-2)/0.2)*.15)*0.2= fabs(0.6)rad=34deg
	 * but this is not a typical case scenario. Usually the error is around 0.01-0.05rad for going straight.
	 * Since the angle is small, we want to keep Kp_outer around a plausible angle value. Distances will usually be +- 0.1m
	 * so the angle change will be in the range of 0.025-0.001 rad/s
	 * @param e distance error
	 * @returns desired angle
	 */
	float outer_loop(float e);


	void reset(){
		L_gain=1.0f;
		R_gain=1.0f;
		reset_error();
	}

	void reset_error(){
		integral=0;
		prev_error=0;
	}

};	




/**
* Customizable class, for changing goals.
*/
struct GoalChanger{
	/**
	* Customizable, for changing goals.
	* @param  control goal pointer
	*/
	virtual Task change_goal(const Task & t)=0;
};
#endif