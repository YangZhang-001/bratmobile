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
    public:

	Task::Action action;

};	

/**
* Customizable, for changing goals.
* arg: control goal pointer
*/
struct GoalChanger{
	virtual void change_goal(Task *);
};
#endif