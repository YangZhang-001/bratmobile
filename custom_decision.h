#include "configurator.h"

/**
 * @brief This header contains callbacks pertaining to the decision-making process which are shared between robot and simulation experiments
 * 
 */

/**
 * @brief Chooses tasks based on a plan
 * 
 */
class Wise_Controller: public Controller{
    public:

	Wise_Controller()=default;

    void next_task(Task & currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan){
	if (plan.empty()){
		//printf("I DON'T KNOW WHAT TO DO NOW\n");
		currentTask=Task(controlGoal.disturbance, UNDEFINED);
		currentTask.action.setLWheelSpeed(0);
		currentTask.action.setRWheelSpeed(0);
		currentTask.change=1;
		return;
	}
	int i=to_task_end(g, plan);
	currentTask = task_to_execute(plan, g, i, controlGoal, currentTask,current_vertices);	
	current_vertices=std::vector(plan.begin(), plan.begin()+i);
	plan.erase(plan.begin(), plan.begin()+i);

    }

/**
 * @brief returns last vertex of the task starting at plan[0]
 * 
 * @param g the cognitive map
 * @param plan the plan
 * @return int iterator to the next vertex not belonging to the task to be executed immediately
 */
    int to_task_end(const TransitionSystem& g, std::vector<vertexDescriptor>& plan){
	int i=0;
	Direction d=g[plan[i]].direction;
	do{
		i++;
	}while(i<plan.size() && g[plan[i]].direction==d && g[plan[0]].Di==g[plan[i]].Di);
	return i;
}


/**
 * @brief Creates a new task by interpreting the states in a plan. Tasks split in several states are treated like one task,
 *  and Tasks terminating in collisions are morphed into tasks aimed at reaching a certain distance from an obstacle
 * 
 * @param p 
 * @param g 
 * @param end_it 
 * @param controlGoal 
 * @param currentTask 
 * @param current_vertices 
 * @return Task 
 */
Task task_to_execute(const std::vector<vertexDescriptor>&p, const TransitionSystem& g,  int end_it, const Task & controlGoal, Task & currentTask, const std::vector<vertexDescriptor> & current_vertices){
	Task t=controlGoal;
	if (p.empty()){
		return t;
	}	
	end_it--;
	b2Transform start_to_end= g[p[0]].start - g[p[end_it]].endPose;
	if (Disturbance Dn= g[p[0]].Dn; Dn.getAffIndex()==AVOID && g[p[0]].direction==DEFAULT){
		Dn.set_affordance(PURSUE);
		t=Task(Dn, g[p[0]].direction, b2Transform_zero, true);
		float distance = g[p[end_it]].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance)); //set task to get within a certain distance from an object (as planned) and then terminate
        disturbance_q=g[p[0]].Dn;

	}
	else{
		Disturbance Di;
		vertexDescriptor currentVertex=0;
		if (!current_vertices.empty()){
			currentVertex=current_vertices[0];
		}
		if (g[p[0]].Di==g[currentVertex].Di){
			Di=currentTask.disturbance;
		}
		else{
			Di=g[p[0]].Di;
            Di.bf.pose=g[p[0]].start_from_Di();
		}
		t=Task(Di, g[p[0]].direction, b2Transform_zero, true);
		// t.endCriteria.angle.set(atan2(end_from_Di.p.y, end_from_Di.p.x));
        disturbance_q=g[p[0]].Di;	
    }

	debug::print_pose(t.disturbance.pose(), "new task disturbance is at: ");
	t.motorStep=motor_step(t.getAction(), start_to_end.p.Length());
	printf("new disturbance x=%f \t y=%f \t %theta=%f\n", t.disturbance.pose().p.x, t.disturbance.pose().p.y, t.disturbance.pose().q.GetAngle() );
	return t;

}

};

 /**
  * @brief Chooses the next task reactively (Braitenberg controller)
  * 
  */
class Reactive_Controller : public Controller{
    public:

	Reactive_Controller()=default;

    void next_task(Task & currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan){
        vertexDescriptor currentVertex=current_vertices[0];
        if (g[currentVertex].Dn.isValid()){
            printf("avoid!");
            currentTask= Task(g[currentVertex].Dn, DEFAULT); //reactive
        }
        else{
            currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
        }
        currentTask.motorStep = motor_step(currentTask.getAction());
        printf("changed to %f\n", currentTask.action.getOmega());

    }


};