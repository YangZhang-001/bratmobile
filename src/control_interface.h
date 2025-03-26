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
	float L=0, R=0;
    public:

	void getData(const Task::Action &a){
		setReady(0);
		L=a.L;
		R=a.R;
		setReady(1);
	}

	float get_L(){
		return L;
	}

	float get_R(){
		return R;
	}


};	

/**
* Customizable, for changing goals.
* arg: control goal pointer
*/
struct GoalChanger{
	virtual void change_goal(Task *);
};
#endif