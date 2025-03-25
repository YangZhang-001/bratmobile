#include "control_interface.h"

void Motor_IO::track_task_execution(){
	//b2Transform deltaPose=worldBuilder.wb_bridger.get_transform(&t, data2fp); //track using obstacle OR dead reckoning
	//here can insert something for wb.bridger, wheel speed control (for step)
	//BodyFeatures *obs_bf;
	b2Transform deltaPose_10Hz=deltaPose;
	deltaPose.p.x/=2;
	deltaPose.p.y/=2;
	deltaPose.q.Set(deltaPose.q.GetAngle()/2);
	task.endCriteria.adjust(deltaPose_10Hz); //adjusting in task so system can be memoryless
	math::applyAffineTrans(-deltaPose_10Hz, &goal); //only update goal
	//math::applyAffineTrans(deltaPose_10Hz, task); 
	bool ended=false;
	// if (wb==NULL){ended=(t.checkEnded(b2Transform_zero)).ended;}
	// else{
	// Disturbance	d_obs=Disturbance(*obs_bf);
	ended=task.checkEnded(task_sensor, b2Transform_zero, &task.disturbance); //the sensor moves with the robot
//	}
	if(task.motorStep==0 || ended){
		task.change=1;
	}
	//estimate_current_vertex(g, t,v);
}

void Motor_IO::change_task(bool b, const std::vector<State>&pv){
	// printf("moving edge = %i -> %i exists %i\n", movingEdge.m_source, movingEdge.m_target, boost::edge(movingEdge.m_source, movingEdge.m_target, transitionSystem).second);
	// printpf("current edge = %i -> %i exists %i\n", currentEdge.m_source, currentEdge.m_target, boost::edge(currentEdge.m_source, currentEdge.m_target, transitionSystem).second);
	if (!b){
		// boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		return;
	}
	if (PLANNING){
		if (pv.empty()){
			printf("I DON'T KNOW WHAT TO DO NOW\n");
			task=Task(goal.disturbance, UNDEFINED);
			task.action.L=0;
			task.action.R=0;
			task.change=1;
			//currentVertex=0; //moving
			return;
		}
		
		// std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, pv[0], transitionSystem);

		printf("erased\n");
		int i=to_task_end();
		//current_vertices=std::vector<vertexDescriptor>(pv.begin(), task_end+1); //sus
       	//currentVertex= *pv.begin();
		task = task_to_execute(pv, i);	
		task_sensor=WorldBuilder::sensor_box(Robot::get_vertices(),b2Transform_zero, &(goal.disturbance));
		// if (g[*task_end].Di==g[currentVertex].Di){
		// 	task_end++;
		// }
		//pv.erase(pv.begin(), task_end);// if (currentTask.action.getLinearSpeed()==0){
		// 	currentTask.motorStep=transitionSystem[currentEdge].step;
		// }
		// else{
		// 	currentTask.motorStep = gt::distanceToSimStep(transitionSystem[currentVertex].distance(), currentTask.action.getLinearSpeed());// 			
		// }
	}
	else{
		if (pv[0].Dn.isValid()){
			task = Task(pv[0].Dn, DEFAULT); //reactive
		}
		// else if(currentTask.direction!=DEFAULT){
		// 	ctask = Task(g[currentVertex].Dn, DEFAULT); //reactive
		// }
		else{
			task = Task(goal.disturbance, DEFAULT); //reactive
		}
		task.motorStep = motor_step(task.getAction());
		//g[movingEdge].step=currentTask.motorStep;
		printf("changed to %f\n", task.action.getOmega());
	}
//	ogStep = currentTask.motorStep;
	return;
}

void Motor_IO::update_graph(TransitionSystem&g, const b2Transform & deltaPose, Task* t, Task * controlGoal){
	math::applyAffineTrans(deltaPose, g);
	math::applyAffineTrans(-deltaPose, controlGoal);
	math::applyAffineTrans(deltaPose, t->start); //d update happens in get_transform
}

int Motor_IO::motor_step(Task::Action a){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		}
		else if (a.getLinearSpeed()>0){
			result = (simulationStep)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
    }


Task Motor_IO::task_to_execute(const std::vector<State>& p, int i){
	Task t=goal;
	if (Disturbance Dn= p[0].Dn; Dn.getAffIndex()==AVOID && p[0].direction==DEFAULT){
		//Disturbance Di= Dn;
		Dn.set_affordance(PURSUE);
		t=Task(Dn, p[0].direction, b2Transform_zero, true);
		float distance = p[i-1].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance)); //set task to get within a certain distance from an object (as planned) and then terminate
	}
	else{
		t=Task(p[0].Di, p[0].direction, b2Transform_zero, true);

	}
	return t;

}



int Motor_IO::to_task_end(){
	int i=0;
	Direction d=plan[i].direction;
	while(i<plan.size() &&plan[i].direction==d){
		i++;
	}
	return i;
	
}
