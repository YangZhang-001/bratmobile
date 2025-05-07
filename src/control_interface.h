#ifndef CONTROL_IF_H
#define CONTROL_IF_H

#include "worldbuilder.h"
#include "task.h"

class Configurator; 

/**
* Input/Output interface for Configurator
*/
class IOInterface{
	protected:
	bool ready=false;
	public:
	IOInterface()=default;

	bool isReady(){
		return ready;
	}

	void setReady(bool b){
		ready=b;
	}

};

/**
* Receives LIDAR data
*/
class LIDAR_In:public IOInterface{ 
public:
	bool debugOn=0;
	int iteration=0;
	bool stop=0;
	CoordinateContainer data2fp;
};

/**
* Output from Configurator to Motors
*/
class Motor_Out:public IOInterface { 
	float L=0, R=0, L_gain=1.0f, R_gain=1.0f, alpha_p=0.01, alpha_i=0.02, alpha_d=0.000;
	float prev_error=0;
	float integral=0;
    public:

	void getData(const Task::Action &a){
		setReady(0);
		L=a.getLWheelSpeed();
		R=a.getRWheelSpeed();
		setReady(1);
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
	*Adjusts gain to R/L wheel
	*/
	void adjust_gain(b2Rot e, b2Rot rot){ //delta rule ()
		integral+=e.GetAngle();
		float increment=alpha_p*e.GetAngle() + alpha_i*integral +alpha_d*prev_error-e.GetAngle();
		//  if (e.GetAngle()>0){
			L_gain+=increment/2;
//		}
	//	else if (e.GetAngle()<0){
			R_gain-=increment/2;
	//	}
		prev_error=e.GetAngle();
	}

	void reset(){
		L_gain=1.0f;
		R_gain=1.0f;
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
	virtual void change_goal(Task * t);
};
#endif