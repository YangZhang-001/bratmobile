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
    float simulationStep=BOX2DRANGE;
    std::vector <vertexDescriptor> plan, current_vertices;
	Task task, goal;
	b2Transform deltaPose=b2Transform_zero;
	bool running=false;

	void track_task_execution(); //returns observed disturbance

	void change_task(bool, std::vector <vertexDescriptor>&, TransitionSystem&, const Task & controlGoal, Task &currentTask, vertexDescriptor & currentVertex);

	int motor_step(Task::Action a);

	void update_graph(TransitionSystem&, const b2Transform & _deltaPose, Task* t, Task * controlGoal);

	//merge vertices into a single task
	Task task_to_execute(const TransitionSystem &, const vertexDescriptor&, const Task& goal);

	vertexDescriptor estimate_current_vertex(TransitionSystem&, Task& currentTask, vertexDescriptor currentVertex);

	void makeRobotSensor(TransitionSystem&, const vertexDescriptor&, const Task& t); //sensor but not linked to a body

	private:
	b2PolygonShape task_sensor;
};	

#endif