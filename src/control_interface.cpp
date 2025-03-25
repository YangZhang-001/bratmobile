#include "control_interface.h"

void ControlInterface::track_task_execution(){
	//b2Transform deltaPose=worldBuilder.wb_bridger.get_transform(&t, data2fp); //track using obstacle OR dead reckoning
	//here can insert something for wb.bridger, wheel speed control (for step)
	//BodyFeatures *obs_bf;
	b2Transform deltaPose_10Hz=deltaPose;
	deltaPose.p.x/=2;
	deltaPose.p.y/=2;
	deltaPose.q.Set(deltaPose.q.GetAngle()/=2);
	task.endCriteria.adjust(deltaPose_10Hz); //adjusting in task so system can be memoryless
	math::applyAffineTrans(-deltaPose_10Hz, goal); //only update goal
	//math::applyAffineTrans(deltaPose_10Hz, task); 
	bool ended=false;
	// if (wb==NULL){ended=(t.checkEnded(b2Transform_zero)).ended;}
	// else{
	Disturbance	d_obs=Disturbance(*obs_bf);
		ended=t.checkEnded(task_sensor, b2Transform_zero, &d_obs); //the sensor moves with the robot
//	}
	if(t.motorStep==0 || ended){
		t.change=1;
	}
	//estimate_current_vertex(g, t,v);
}

void ControlInterface::change_task(bool b, std::vector <vertexDescriptor>& pv, TransitionSystem & g, const Task & controlGoal, Task &currentTask, vertexDescriptor & currentVertex){
	// printf("moving edge = %i -> %i exists %i\n", movingEdge.m_source, movingEdge.m_target, boost::edge(movingEdge.m_source, movingEdge.m_target, transitionSystem).second);
	// printpf("current edge = %i -> %i exists %i\n", currentEdge.m_source, currentEdge.m_target, boost::edge(currentEdge.m_source, currentEdge.m_target, transitionSystem).second);
	if (!b){
		// boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		return;
	}
	if (PLANNING){
		if (pv.empty()){
			printf("I DON'T KNOW WHAT TO DO NOW\n");
			currentTask=Task(controlGoal.disturbance, UNDEFINED);
			currentTask.action.L=0;
			currentTask.action.R=0;
			currentTask.change=1;
			//currentVertex=0; //moving
			return;
		}
		
		// std::pair<edgeDescriptor, bool> ep=boost::add_edge(currentVertex, pv[0], transitionSystem);

		printf("erased\n");
		// transitionSystem[movingVertex].Di=transitionSystem[currentVertex].Di;
		// transitionSystem[movingVertex].outcome=simResult::successful;
		// movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		//printf("added edge %i->%i\n", movingVertex, currentVertex);
		// boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		//printf("removed out edges of moving v\n");
		// if (ep.first.m_source==ep.first.m_target){
		// 	currentEdge=movingEdge;
		// }
		// else{
		// 	currentEdge=ep.first;
		// }
		// transitionSystem[movingEdge].direction=transitionSystem[ep.first].direction;
		// transitionSystem[movingEdge].step=currentTask.motorStep;
		auto nextEdge=boost::edge(currentVertex, pv[0], g);
		try{
			if (!nextEdge.second){
				//throw std::invalid_argument("no edge between current v and next in plan!");
				throw(nextEdge.first);
			}			
		}
		catch (edgeDescriptor null_edge){
			boost::add_edge(currentVertex, pv[0], g);
		}
		std::vector<vertexDescriptor>::iterator task_end=gt::to_task_end(nextEdge.first, g, pv, pv.begin());
		current_vertices=std::vector<vertexDescriptor>(pv.begin(), task_end+1); //sus
       	currentVertex= *pv.begin();
		currentTask = task_to_execute(g, currentVertex, controlGoal);	
		task_sensor=WorldBuilder::sensor_box(Robot::get_vertices(),b2Transform_zero, &(controlGoal.disturbance));
		if (g[*task_end].Di==g[currentVertex].Di){
			task_end++;
		}
		pv.erase(pv.begin(), task_end);// if (currentTask.action.getLinearSpeed()==0){
		// 	currentTask.motorStep=transitionSystem[currentEdge].step;
		// }
		// else{
		// 	currentTask.motorStep = gt::distanceToSimStep(transitionSystem[currentVertex].distance(), currentTask.action.getLinearSpeed());// 			
		// }
	}
	else{
		if (g[currentVertex].Dn.isValid()){
			currentTask = Task(g[currentVertex].Dn, DEFAULT); //reactive
		}
		else if(currentTask.direction!=DEFAULT){
			currentTask = Task(g[currentVertex].Dn, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}
		gt::fill(simResult(), g[currentVertex].ID);
		currentTask.motorStep = motor_step(currentTask.getAction());
		//g[movingEdge].step=currentTask.motorStep;
		printf("changed to %f\n", currentTask.action.getOmega());
	}
//	ogStep = currentTask.motorStep;
	return;
}

void ControlInterface::update_graph(TransitionSystem&g, const b2Transform & deltaPose, Task* t, Task * controlGoal){
	math::applyAffineTrans(deltaPose, g);
	math::applyAffineTrans(-deltaPose, controlGoal);
	math::applyAffineTrans(deltaPose, t->start); //d update happens in get_transform
}

int ControlInterface::motor_step(Task::Action a){
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


Task ControlInterface::task_to_execute(const TransitionSystem & g, const vertexDescriptor& v, const Task& controlGoal){
	Task t=controlGoal;
	if (Disturbance Dn= g[v].Dn; Dn.getAffIndex()==AVOID && g[v].direction==DEFAULT){
		//Disturbance Di= Dn;
		Dn.affordanceIndex=PURSUE;
		t=Task(Dn, g[v].direction, b2Transform_zero, true);
		float distance = g[v].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance)); //set task to get within a certain distance from an object (as planned) and then terminate
	}
	else{
		t=Task(g[v].Di, g[v].direction, b2Transform_zero, true);

	}
	return t;

}

vertexDescriptor ControlInterface::estimate_current_vertex(TransitionSystem& g, Task& currentTask, vertexDescriptor currentVertex){
	vertexDescriptor task_start;
	try {
		task_start=current_vertices.at(0);
	}
	catch(const std::out_of_range& oor){
		printf("current vertices empty\n");
		return currentVertex;
	}
	//State q(b2Transform_zero, currentTask.disturbance, currentTask.direction);
	// if ((g[task_start].Dn.getAffIndex()==AVOID && currentTask.disturbance.affordanceIndex==PURSUE)){
	// 	//return 0;//moving
	// 	q.Di=currentTask.disturbance;
	// 	q.Di.affordanceIndex=AVOID;
	// 	q.Dn=Disturbance();
	// }
	b2Transform Di_distance=currentTask.from_Di(), v_from_D=b2Transform_zero;

	float sum=10000;
	StateMatcher matcher;
//	State q(b2Transform_zero, currentTask.disturbance, currentTask.direction);
	for (vertexDescriptor & v:current_vertices){
		if ((g[task_start].Dn.getAffIndex()==AVOID && currentTask.disturbance.affordanceIndex==PURSUE)){
			v_from_D=g[v].start_from_Dn();
		}
		else{
			v_from_D=g[v].start_from_Di();
		}
		b2Transform transform_diff=Di_distance-v_from_D;
		float sum_diff=fabs(transform_diff.p.x+transform_diff.p.y+transform_diff.q.GetAngle());
		if (sum_diff<sum){
			currentVertex=v;
			sum=sum_diff;
		}				
	}
	return currentVertex;

}
