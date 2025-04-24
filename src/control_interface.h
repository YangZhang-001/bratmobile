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
	float L=0, R=0, L_gain=1, R_gain=1, alpha=0.001;
    public:

	void getData(const Task::Action &a){
		setReady(0);
		L=a.L;
		R=a.R;
		setReady(1);
	}

	float get_L(){
		return L*L_gain;
	}

	float get_R(){
		return R*R_gain;
	}

	/**
	*Adjusts gain to R/L wheel
	*/
	void adjust_gain(b2Rot e, b2Rot rot){ //delta rule ()
		float increment=alpha*e.GetAngle()*rot.GetAngle();
		if (e.GetAngle()<0.05){
			L_gain+=increment;
		}
		else if (e.GetAngle()>0.05){
			R_gain+=increment;
		}
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