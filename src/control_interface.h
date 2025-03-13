#ifndef CONTROL_IF_H
#define CONTROL_IF_H

#include "worldbuilder.h"
#include "task.h"
//extern std::mutex ctr_mutex;

class ControlInterface{ //tracks task execution
    public:
    float simulationStep=BOX2DRANGE;
    std::vector <vertexDescriptor> plan, current_vertices;

	ControlInterface(){}

	void track_task_execution(Task &, TransitionSystem&, Task * controlGoal, vertexDescriptor &v);

	void change_task(bool, std::vector <vertexDescriptor>&, TransitionSystem&, const Task & controlGoal, Task &currentTask, vertexDescriptor & currentVertex);

	int motor_step(Task::Action a);

	void update_graph(TransitionSystem&, const b2Transform & _deltaPose, Task* t, Task * controlGoal);

	//merge vertices into a single task
	Task task_to_execute(const TransitionSystem &, const vertexDescriptor&, const Task& t);

	vertexDescriptor estimate_current_vertex(TransitionSystem&, Task& currentTask, vertexDescriptor currentVertex);

	void makeRobotSensor(TransitionSystem&, const vertexDescriptor&, const Task&); //sensor but not linked to a body

};	

#endif