#include "control_interface.h"

void ControlInterface::track_task_execution(Task & t){
	//b2Transform deltaPose=worldBuilder.wb_bridger.get_transform(&t, data2fp); //track using obstacle OR dead reckoning
	//here can insert something for wb.bridger, wheel speed control (for step)
	b2Transform deltaPose=t.action.getTransform(MOTOR_CALLBACK);
	// printf("shift graph by:\n");
	// debug::print_pose(deltaPose);
	//adjust_rw_task(movingVertex, transitionSystem, &t, deltaPose); //readjust end criteria
	update_graph(transitionSystem, deltaPose, t);//lateral error is hopefully noise and is ignored
	//TO REMOVE ONCE YOU APPY FULLY CLOSED LOOP INSTEAD OF DEAD RECKONING
	math::applyAffineTrans(deltaPose, t.disturbance); //remove later
	//debug::print_pose(t.disturbance.pose(), "TASK DISTURBANCE IS");
	//debug::print_pose(t.start, "TASK start IS");
	// bool (t.checkEnded()).ended;s
	// debug::print_pose(t.from_Di(b2), "TASK start IS");
	//printf("in track: end criteria d= %f ",t.endCriteria.distance.get());
	//t.motorStep--; //delete this
	bool ended=(t.checkEnded(b2Transform_zero)).ended;
	if(t.motorStep==0 || ended){
		t.change=1;
	}

}

std::vector <vertexDescriptor> ControlInterface::change_task(bool b, std::vector <vertexDescriptor> pv, const TransitionSystem & g, const Task & controlGoal, Task &currentTask){
	// printf("moving edge = %i -> %i exists %i\n", movingEdge.m_source, movingEdge.m_target, boost::edge(movingEdge.m_source, movingEdge.m_target, transitionSystem).second);
	// printpf("current edge = %i -> %i exists %i\n", currentEdge.m_source, currentEdge.m_target, boost::edge(currentEdge.m_source, currentEdge.m_target, transitionSystem).second);
	if (!b){
		// boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		return pv;
	}
	if (planning){
		if (pv.empty()){
			printf("I DON'T KNOW WHAT TO DO NOW\n");
			currentTask=Task(controlGoal.disturbance, UNDEFINED);
			currentTask.action.L=0;
			currentTask.action.R=0;
			currentTask.change=1;
			//currentVertex=movingVertex;
			return pv;
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
		if (!nextEdge.second){
			throw std::invalid_argument("no edge between current v and next in plan!");
		}
		std::vector<vertexDescriptor>::iterator task_end=to_task_end(nextEdge.first, transitionSystem, pv, pv.begin());
		current_vertices=std::vector<vertexDescriptor>(pv.begin(), task_end);
        currentVertex= *task_end;
		currentTask = task_to_execute(transitionSystem, currentVertex);		
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
		if (transitionSystem[currentVertex].Dn.isValid()){
			currentTask = Task(transitionSystem[currentVertex].Dn, DEFAULT); //reactive
		}
		else if(currentTask.direction!=DEFAULT){
			currentTask = Task(transitionSystem[currentVertex].Dn, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}
		gt::fill(simResult(), &transitionSystem[currentVertex]);
		currentTask.motorStep = motorStep(currentTask.getAction());
		transitionSystem[movingEdge].step=currentTask.motorStep;
		printf("changed to %f\n", currentTask.action.getOmega());
	}
//	ogStep = currentTask.motorStep;
	return pv;
}

void ControlInterface::update_graph(TransitionSystem&g, const b2Transform & deltaPose, Task* t){
	math::applyAffineTrans(deltaPose, g);
	math::applyAffineTrans(-deltaPose, &controlGoal);
	math::applyAffineTrans(deltaPose, t->start); //d update happens in get_transform
}

int ControlInterface::motorStep(Task::Action a){
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
	if (Disturbance Dn= g[v].Dn; Dn.getAffIndex()==AVOID){
		Disturbance Di= Dn;
		Di.affordanceIndex=PURSUE;
		t=Task(Di, g[v].direction, b2Transform_zero, true);
		float distance = g[v].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance));
	}
	else{
		t=Task(g[v].Di, g[v].direction, b2Transform_zero, true);

	}
	return t;

}
