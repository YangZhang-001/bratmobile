#ifndef CONTROL_IF_H
#define CONTROL_IF_H

#include "worldbuilder.h"
#include "task.h"
#include <mutex>

//std::mutex mutex;

class ControlInterface{ //tracks task execution
    public:
    float simulationStep=BOX2DRANGE;
    std::vector <vertexDescriptor> plan, current_vertices;

	ControlInterface(){}

	void track_task_execution(Task &, TransitionSystem&);

	void change_task(bool, std::vector <vertexDescriptor>&, TransitionSystem&, const Task & controlGoal, Task &currentTask, const vertexDescriptor & currentVertex);

	int motor_step(Task::Action a);

	void update_graph(TransitionSystem&, const b2Transform & _deltaPose, Task* t, Task * controlGoal);

	Task task_to_execute(const TransitionSystem &, const vertexDescriptor&, const Task& t);


};

#endif