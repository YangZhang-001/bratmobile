#include "attentive.h"

bool AttentiveConfigurator::recycle_plan(vertexDescriptor v, vertexDescriptor &v0, vertexDescriptor & task_start, StateMatcher::MATCH_TYPE& matchType, 
											b2Transform & shift_start, b2Transform& sk_first_start, std::pair<edgeDescriptor, bool>&edge, 
											std::vector<vertexDescriptor> &plan_prov, Direction t_get_direction){
	bool finished=false, result=false;
	bool been = matchType==StateMatcher::ABSTRACT || matchType==StateMatcher::_TRUE;
	Task controlGoal_adjusted= controlGoal;
	//position of task start with respect to goal disturbance (pov)
//	shift_start= b2MulT(b2MulT(sk_first_start, controlGoal.getStart()), transitionSystem[task_start].start);
	shift_start= b2MulT(b2MulT(sk_first_start, controlGoal.getStart()), transitionSystem[task_start].start);
	Mul(shift_start, controlGoal_adjusted);
	if (edge.first.m_source!=v0){
		edge= gt::add_edge(v0, task_start, transitionSystem, iteration, transitionSystem[edge.first.m_target].direction);
	}
	if (!boost::edge(edge.first.m_source, edge.first.m_target, transitionSystem).second){
		return false;
	}
	transitionSystem[edge.first].enableOverride();	
	ExecutionInfo info=package_info(TransitionSystem::null_vertex(), been);
	info.overarchingGoal(controlGoal_adjusted); 
	auto plan_tmp=planner->plan(transitionSystem, v, info, &finished); //not v but task start
	//printf("out of explore planner\n");
	if (finished){
		plan_prov=plan_tmp;
		result=true;
		if (plan_prov.empty()){ // task_start==currentVertex in\tead of pv empty
			plan_prov.insert(plan_prov.begin(), task_start);
		}
		if (t_get_direction== transitionSystem[task_start].direction){
			transitionSystem[v0].options.clear();
		}
		else{
			transitionSystem[v0].options={transitionSystem[task_start].direction};
		}
	}
	return result;
}

VertexMatch AttentiveConfigurator::findMatch(State s, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd, vertexDescriptor src){
	bool currentTaskOK=s.start==b2Transform_zero && s.direction==currentTask.get_direction() && 
						!hasPlanFinished()&& s.Dn.getAffIndex()==transitionSystem[currentVertex].Dn.getAffIndex(); //this state represent current task
	if (currentTaskOK){ //if the state to be matched is the current one, return it
		return VertexMatch(StateMatcher::_TRUE, currentVertex);
	}
	else if(std::pair<bool, vertexDescriptor> isOk= isPlannedTaskOK(src, s); isOk.first &&  !hasPlanFinished()){
		return VertexMatch(StateMatcher::_TRUE, isOk.second);
	}
	return hardMatch(s, dir, match_type, _sd);
}

std::pair<bool, vertexDescriptor> AttentiveConfigurator::isPlannedTaskOK(vertexDescriptor src, State s){
	std::pair<bool, vertexDescriptor>result(false, TransitionSystem::null_vertex());	
	if (src!=TransitionSystem::null_vertex()){
		auto srcIt=std::find(m_plan.begin(), m_plan.end(), src);
		bool srcIsInPlan=srcIt!=m_plan.end();
		bool srcIsLast=srcIt==(m_plan.end()-1);		
		vertexDescriptor v1=TransitionSystem::null_vertex();
		if (srcIsInPlan){
			if (!srcIsLast){
				v1=*(srcIt+1);
			}
			else{
				v1=src;
			}
			if (s.direction== transitionSystem[v1].direction && s.Dn.getAffIndex()==transitionSystem[v1].Dn.getAffIndex() ){
				result.first=true;
				result.second= v1;
			}
		}
	}
	return result;

}




Disturbance DiscreteConfigurator::getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start){
    return g[v].Dn;
}

Robot DiscreteConfigurator::makeRobot(b2World & world, const Task & task){
	Robot robot=Configurator::makeRobot(world, task);
	return robot;
}

float DiscreteConfigurator::remainingSimulationTime(const Task *const t){
    if (t && get_start(t)==b2Transform_zero && get_direction(t)==currentTask.get_direction() && iteration>1){
        b2Transform remainingTransform= transitionSystem[currentVertex].endPose;
        return 	Controller::motor_step(t->getAction(), remainingTransform.p.Length())*MOTOR_CALLBACK;
    }
    else if (get_direction(t)==DEFAULT){
        return simulationStep/ t->getAction().getLinearSpeed();
    }
    return Configurator::remainingSimulationTime();
}

VertexMatch DiscreteConfigurator::findMatch(State s, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd,vertexDescriptor src){
	if (s.start==b2Transform_zero && s.direction==currentTask.get_direction() && 
			 s.outcome==simResult::successful && iteration>1){ //if the state to be matched is the current one, return it
		return VertexMatch(StateMatcher::_TRUE, currentVertex);
	}
	return hardMatch(s, dir, StateMatcher::_TRUE, _sd);
}

void DiscreteConfigurator::transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src) {
	FocusedConfigurator::transitionMatrix(v, d, src);
	Task temp(controlGoal.get_disturbance(), DEFAULT, transitionSystem[v].endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	auto oe=gt::outEdges(transitionSystem, v, d);
	bool executingThisTask=( !currentTask.get_change()|| !oe.empty()) && (iteration>1) && transitionSystem[v].options.size()>1;
	//add left/right transitions by default to straight tasks
	if (!isTurning(d) &&!executingThisTask && temp.getAction().getOmega()==0 && transitionSystem[v].outcome==simResult::successful){
		int random= rand();
		if (random%2==0){
			transitionSystem[v].options.push_back(LEFT);
			transitionSystem[v].options.push_back(RIGHT);
		}
		else{
			transitionSystem[v].options.push_back(RIGHT);
			transitionSystem[v].options.push_back(LEFT);// = {DEFAULT, LEFT, RIGHT};
		}	
	}
	//allow for tasks which have been encountered before and have oe nonempty but not visited
	//to explore driving in DEFAULT ahead (prevented in AttentiveConfigurator::transitionMatrix)
	// if (std::pair<bool, edgeDescriptor> ve=gt::visitedEdge(oe, transitionSystem, currentVertex);
	// 		executingThisTask && !ve.first){
	// 			transitionSystem[v].options.emplace(transitionSystem[v].options.begin(), DEFAULT);
	// 		}
	
}

bool DiscreteConfigurator::closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v){
	closed.emplace(v);
	return true;
}

void DiscreteConfigurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src, vertexDescriptor startRecycle){
	for (vertexDescriptor v:evaluation_q){
		if (!isTurning(transitionSystem[v].direction)){
			addToPriorityQueue(v, priority_q, closed);
		}
		auto likelyEdge=gt::getMostLikely(transitionSystem, inEdges(v), iteration);
		if (likelyEdge.first){
			if (likelyEdge.second.m_source==MOVING_VERTEX && transitionSystem[v].direction==currentTask.get_direction() && transitionSystem[v].outcome==simResult::crashed){
				auto moving_it=std::find(closed.begin(), closed.end(), MOVING_VERTEX);
				if (moving_it!=closed.end()){
					closed.erase(moving_it);
					addToPriorityQueue(MOVING_VERTEX, priority_q, closed);
					applyTransitionMatrix(MOVING_VERTEX, transitionSystem[v].direction, false, MOVING_VERTEX, plan_prov);
				}
			}
		}
	}
	evaluation_q.clear();
}

std::vector<Direction> DiscreteConfigurator::partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve){
	std::vector <Direction> result=FocusedConfigurator::partiallyExplorativeOptions(ve);
	if (ve.first){
		if (!transitionSystem[ve.second.m_target].visited() && transitionSystem[ve.second.m_source].outcome!=simResult::crashed){
			if (!isTurning(transitionSystem[ve.second.m_target].direction)){
			result={DEFAULT, LEFT, RIGHT};
			}
			else{
				result={DEFAULT, transitionSystem[ve.second.m_target].direction};
			}
		}
	}
	return result;
}

bool DiscreteConfigurator::shouldPartiallyExplore(const std::vector<edgeDescriptor>& oe, std::pair<bool, edgeDescriptor> ve){
	return ( !currentTask.get_change() ||!oe.empty()) && iteration>1 && ve.first;
}



bool DiscreteConfigurator::propagateD(vertexDescriptor v1, vertexDescriptor v0){
	while(FocusedConfigurator::propagateD(v1, v0)){
		v1=v0;
		auto ve= gt::visitedEdge(inEdges(v1, DEFAULT),transitionSystem, currentVertex);
		auto dummyEdge=boost::edge(DUMMY, v1, transitionSystem);
		if (!ve.first && !dummyEdge.second){
			return false;	
		}
		else{
			struct TrueEdge{
				bool operator()(const std::pair<bool, edgeDescriptor>& p1, const std::pair<bool, edgeDescriptor> &p2)const{
					return p1.second<p2.second;
				}
			};
			ve=std::max(ve, std::pair<bool, edgeDescriptor>(dummyEdge.second, dummyEdge.first), TrueEdge());
		}
		v0=ve.second.m_source;
		EndedResult er=estimateCost(transitionSystem[v0],transitionSystem[v0].start, transitionSystem[v0].direction, controlGoal);
		transitionSystem[v0].phi=evaluationFunction(er, v0, m_plan);
	}
	return false;
}

bool DiscreteConfigurator::canPropagate(vertexDescriptor v){
	return transitionSystem[v].direction==STOP || transitionSystem[v].direction==DEFAULT;
}

bool DiscreteConfigurator::canReassignOutcome(vertexDescriptor v){
	return !transitionSystem[v].isTurning();
}