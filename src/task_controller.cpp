#include "task_controller.h"

int Controller::motor_step(Task::Action a, float distance){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		}
		else if (a.getLinearSpeed()>0){
			result = (distance)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
}

Task Controller::stopTask(const Task& controlGoal){
	Task currentTask(controlGoal.get_disturbance(), UNDEFINED);
	currentTask.getAction().setLWheelSpeed(0);
	currentTask.getAction().setRWheelSpeed(0);
	currentTask.set_change(true);
	return currentTask;
}


Task Wise_Controller::next_task(Task currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan){
	if (plan.empty()){
	//printf("I DON'T KNOW WHAT TO DO NOW\n");
	current_vertices={MOVING_VERTEX};
	return stopTask(controlGoal);
	}
	int i=to_task_end(g, plan);
	currentTask = task_to_execute(plan, g, i, controlGoal, currentTask,current_vertices);	
	current_vertices=std::vector(plan.begin(), plan.begin()+i);
	plan.erase(plan.begin(), plan.begin()+i);
	return currentTask;
}

int Wise_Controller::to_task_end(const TransitionSystem& g, std::vector<vertexDescriptor>& plan){
	int i=0;
	Direction d=g[plan[i]].direction;
	do{
		i++;
	}while(i<plan.size() && g[plan[i]].direction==d && g[plan[0]].Di==g[plan[i]].Di);
	return i;
}

Task Wise_Controller::task_to_execute(const std::vector<vertexDescriptor>&p, const TransitionSystem& g,  int end_it, const Task & controlGoal, Task & currentTask, const std::vector<vertexDescriptor> & current_vertices){
	Task t=controlGoal;
	if (p.empty()){
		return t;
	}	
	end_it--;
	//b2Transform start_to_end= g[p[0]].start - g[p[end_it]].endPose;
	b2Transform start_to_end= b2MulT(g[p[0]].start, g[p[end_it]].endPose);
	if (Disturbance Dn= g[p[0]].Dn; Dn.getAffIndex()==AVOID && g[p[0]].direction==DEFAULT){
		Dn.set_affordance(PURSUE);
		t=Task(Dn, g[p[0]].direction, b2Transform_zero, true);
		float distance = g[p[end_it]].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance)); //set task to get within a certain distance from an object (as planned) and then terminate
        disturbance_q=g[p[0]].Dn;
	}
	else{
		Disturbance Di;
		vertexDescriptor currentVertex=get_current_vertex(current_vertices);
		if (g[p[0]].Di==g[currentVertex].Di){
			Di=currentTask.get_disturbance();  //set disturbance where it already is (has been tracked before)
		}
		else{
			Di=g[p[0]].Di;
            Di.bf.pose=g[p[0]].start_from_Di(); //set disturbance where it is EXPECTED to be (to generate error signal)
		}
		t=Task(Di, g[p[0]].direction, b2Transform_zero, true);
		if (Di.getAffIndex()==PURSUE ){
			if (g[p[0]].isTurning()){
				Angle angle(atan(g[p[0]].end_from_Di().p.y/ g[p[0]].end_from_Di().p.x));
				t.setEndCriteria(angle);
			}
			else if(!g[p[0]].isTurning()){
				Distance distance(g[p[0]].end_from_Di().p.x);
				t.setEndCriteria(distance);
				
			}
		}
        disturbance_q=g[p[0]].Di; 	
    }
	vertexDescriptor plan_end=p[p.size()-1];
	// _D_to_goal=b2MulT(disturbance_q.pose() , g[plan_end].Di.pose()); //assumes that the last step in the plan reaches the goal
	if (controlGoal.get_disturbance().getAffIndex()!=NONE){
		_D_to_goal=b2MulT(disturbance_q.pose() , controlGoal.get_disturbance().pose()); //assumes that the last step in the plan reaches the goal
	}
	t.setMotorStep(motor_step(t.getAction(), start_to_end.p.Length()));
	// printf("new disturbance x=%f \t y=%f \t %theta=%f\n", t.get_disturbance().pose().p.x, t.get_disturbance().pose().p.y, t.get_disturbance().pose().q.GetAngle() );
	// printf("goal x=%f \t y=%f \t %theta=%f\n", g[plan_end].Di.pose().p.x, g[plan_end].Di.pose().p.y, g[plan_end].Di.pose().q.GetAngle() );
	return t;

}


Task Reactive_Controller::next_task( Task currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan){
	vertexDescriptor currentVertex=get_current_vertex(current_vertices);
	if (g[currentVertex].Dn.isValid()){
		printf("avoid!");
		currentTask= Task(g[currentVertex].Dn, DEFAULT); //reactive
	}
	else{
		currentTask = Task(controlGoal.get_disturbance(), DEFAULT); //reactive
	}
	currentTask.setMotorStep(motor_step(currentTask.getAction(), g[currentVertex].endPose.p.Length()));
	printf("changed to %f\n", currentTask.getAction().getOmega());
	return currentTask;

}

Task OpenLoopController::next_task(Task currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan){
	if (plan.empty() && currentTask.is_over()){
		current_vertices={MOVING_VERTEX};
		return stopTask(controlGoal);
	}
	currentTask=Task(g[plan[0]].Di, g[plan[0]].direction, b2Transform_zero, true);
	vertexDescriptor currentVertex=get_current_vertex(current_vertices);
	auto e=boost::edge(currentVertex, plan[0], g);
	current_vertices=std::vector<vertexDescriptor>({plan[0]});
	if (!e.second){
		currentTask.setMotorStep(motor_step(currentTask.getAction(), g[plan[0]].distance()));
	}
	else{
		currentTask.setMotorStep(g[e.first].step);
	}
	plan.erase(plan.begin());
	if (currentTask.is_over()&& currentTask.getAction().getLWheelSpeed()!=0 && currentTask.getAction().getRWheelSpeed()!=0){
		return stopTask(controlGoal);
	}   
	return currentTask;
}