#ifndef CONTROL_IF_H
#define CONTROL_IF_H

#include "worldbuilder.h"
#include "task.h"
//extern std::mutex ctr_mutex;
//#include "configurator.h"
class Configurator; 

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

	//virtual void transfer_data(Configurator *);
};

class LIDAR_In:public IOInterface{ //data interface for configurator
protected:

public:
	bool debugOn=0;
	int iteration=0;
//	bool ready=0;
	bool stop=0;
	CoordinateContainer data2fp;
	//std::vector <vertexDescriptor> plan_on_hold;

	// void setReady(bool b);

	// bool isReady();

	// void transfer_data(Configurator * c){
	// 	c->data2fp=data2fp;
	// }

};

class Motor_IO:public IOInterface{ //tracks task execution
    public:
	Motor_IO(){
		ready=1;
	}
    float simulationStep=BOX2DRANGE;
    std::vector <State> plan;
	Task task, goal;
	b2Transform deltaPose=b2Transform_zero;
	bool running=false;
	int plan_iterator=0, iteration=0;

	void track_task_execution(); //returns observed disturbance

	void change_task(bool b, const std::vector<State>&pv);

	int motor_step(Task::Action a);

	void update_graph(TransitionSystem&, const b2Transform & _deltaPose, Task* t, Task * controlGoal);

	//merge vertices into a single task
	Task task_to_execute(const std::vector<State>&, int);

	void makeRobotSensor(TransitionSystem&, const vertexDescriptor&, const Task& t); //sensor but not linked to a body

	int to_task_end();
	private:
	b2PolygonShape task_sensor;
};	

#endif