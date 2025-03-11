#ifndef CONTROL_IF_H
#define CONTROL_IF_H

#include "graphTools.h"
#include <mutex>

std::mutex mutex;

class ControlInterface{ //tracks task execution
	std::vector <vertexDescriptor> plan, current_vertices;

	public:
    float simulationStep=BOX2DRANGE;

	ControlInterface()=default;

	void track_task_execution(Task &);

	std::vector <vertexDescriptor> change_task(bool, std::vector <vertexDescriptor>, const TransitionSystem&, const Task & controlGoal, Task &currentTask);

	int motor_step(Task::Action a);

	void update_graph(TransitionSystem&, const b2Transform & _deltaPose, Task* t);

	Task task_to_execute(const TransitionSystem &, const vertexDescriptor&, const Task&);


};

#endif