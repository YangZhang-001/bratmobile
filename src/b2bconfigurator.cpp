#include "b2bconfigurator.h"

bool B2BConfigurator::closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v){
	int MAX_OUT=5;
	if (transitionSystem[v].isTurning()){MAX_OUT=2;}
	std::vector<Direction> directions={UNDEFINED};
	if (getExploredDirections(v, directions).size()>=MAX_OUT){
		closed.emplace(v);
		return true;
	}
	return false;

}

// std::vector<Direction> B2BConfigurator::partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve){
// 	std::vector <Direction> result;
// 	if (ve.first){
// 		if(transitionSystem[ve.second.m_target].visited()){
// 			if (transitionSystem[ve.second.m_target].outcome!=simResult::crashed){
// 				return {transitionSystem[ve.second.m_target].direction};
// 			}
// 			else if (transitionSystem[ve.second.m_target].outcome==simResult::crashed){
// 				transitionInHindsight(ve.second.m_source, [&](vertexDescriptor v){ //LAMBDA FUNCTION!
// 					std::vector <Direction> result={DEFAULT, LEFT, RIGHT};
// 					erase_from_vector(result, transitionSystem[v].direction);
// 					return result;
// 				});
// 			}
// 		}
// }
// return result;
// }

std::vector <vertexDescriptor> B2BConfigurator::splitTask(vertexDescriptor v, Direction d, vertexDescriptor src){
    std::vector <vertexDescriptor> split={v};
	auto first_edge=boost::edge(src, v, transitionSystem); //assumes exists
	if (transitionSystem[v].isTurning()){ //d
		if ((src==MOVING_VERTEX || src==DUMMY )&& transitionSystem[v].outcome==simResult::crashed){
			split.emplace(split.begin(), src);
		}
    	return split;
	}
	if (transitionSystem[v].outcome != simResult::crashed){
		return split;
	}
	auto ie=inEdges(src);
	auto sameIterationEdgeIt=check_vector_for(ie, SameIteration(transitionSystem, iteration));
	if (!transitionSystem[src].isTurning()&& !ie.empty()){ //! //&& sameIterationEdgeIt!=ie.end()
		transitionSystem[src].outcome=simResult::safeForNow;
		split.insert(split.begin(), src);
	}
	return split;
}

void B2BConfigurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src, vertexDescriptor startRecycle){
	AttentiveConfigurator::backtrack(evaluation_q, priority_q, closed, plan_prov, module_src, startRecycle);
	for (ClearVoyance::DisturbanceLookahead & dl: clearvoyance.getLookaheads()){
		addToPriorityQueue(dl.source, priority_q, closed);
	}
}

// std::vector <vertexDescriptor> B2BConfigurator::task_vertices( vertexDescriptor v, std::pair<bool, edgeDescriptor>* ep){
// 	std::vector <vertexDescriptor> result= {v};
// 	Direction d=UNDEFINED;
// 	std::pair<bool, edgeDescriptor>ep2(false, edgeDescriptor()), _ep=ep2;
// 	//do {
// 		std::vector <edgeDescriptor> ie=inEdges( v);
// 		ep2= gt::visitedEdge(ie, transitionSystem,v);
// 		if (!ep2.first){
// 			ep2=gt::getMostLikely(transitionSystem, ie, iteration);
// 		}
// 		if (ep2.first){
// 			//if (ep2.second.m_target==v){ //size 1
// 				_ep=ep2; //assign ep to define direction
// 				d= transitionSystem[_ep.second.m_target].direction;
// 				if (ep!=NULL){
// 					transitionSystem[_ep.second].it_observed=iteration;
// 				}
// 				// for (edgeDescriptor e: ie){
// 				// 	if (transitionSystem[e.m_target].direction==d && e!=ep2.second &&
// 				// 		transitionSystem[e.m_source].Di == transitionSystem[_ep.second.m_source].Di &&
// 				// 		transitionSystem[e.m_source].Dn == transitionSystem[_ep.second.m_target].Dn){
// 				// 		ep2.second=e;
// 				// 		break;
// 				// 	}
// 				// }
// 			// }
// 			// else if (transitionSystem[ep2.second.m_target].direction==d &&
// 			//  	transitionSystem[ep2.second.m_target].Di == transitionSystem[_ep.second.m_target].Di &&
// 			//  	transitionSystem[ep2.second.m_target].Dn == transitionSystem[_ep.second.m_target].Dn){ //same task!
// 			// 	result.emplace(result.begin(), ep2.second.m_target); //source
// 			// }
// 		}
// 	if (NULL!=ep){
// 		*ep=_ep;
// 	}
// 	return result;
// }

bool B2BConfigurator::attentionWindowOverlaps(const Disturbance & Di,const State & q, b2World & world, const Disturbance & focus){
	Task task(Di, DEFAULT, q.endPose, true);
	Robot robot(&world);
	robot.body()->SetTransform(task.getStart().p, task.getStart().q.GetAngle());
	b2AABB box =worldBuilder->makeRobotSensor(robot.body(), focus);
	b2Fixture *sensor =GetSensor(robot.body());
	bool overlap=overlaps(robot.body(), &Di) && sensor;
	world_cleanup(world);
	return overlap;

}

int B2BConfigurator::visitedEdgeCount(const std::vector <edgeDescriptor>& es){
	int count=0;
	for (const edgeDescriptor& e: es){
		if (transitionSystem[e].it_observed==iteration){
			count++;
		}
	}
	return count;
}

// bool canGoToClearVoyance(const std::vector <edgeDescriptor> &oe, Direction direction){
// 	// int visitedCount=visitedEdgeCount(oe);
// 	// if (isTurning)

// }

int B2BConfigurator::minimumEdgesForClearvoyance(Direction direction){
	if (isTurning(direction)){
		return 1; //if turning, only one edge is needed to be visited
	}
	return 3; //if not turning, at least two edges are needed to be visited
}




Disturbance B2BConfigurator::getDisturbance(TransitionSystem&g,vertexDescriptor v, b2World & world, const Direction& dir, const b2Transform& start){
	b2Transform invmul=b2help::InvMul(start,g[v].endPose);
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=inEdges(v);
		std::vector <edgeDescriptor> out=gt::outEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);	
		//visitedOrVisitingEdge(out, transitionSystem, currentVertex);
		Disturbance CVDi=clearvoyance.query(v);
		bool notClearVoyance=visitedEdgeCount(out)<minimumEdgesForClearvoyance(g[v].direction);
		if ((visited.first)||out.empty()){ //if edges have not been expanded OR if they were expanded in previous iteration
			if (g[v].Di.isValid() && g[v].Di.getAffIndex()==AVOID && (g[v].direction!=dir || (g[v].isTurning() && isTurning(dir)))){ //if Di is valid and not the same direction as the vertex || (g[v].isTurning() && isTurning(dir))
				Disturbance Di= g[v].Di;
				if (attentionWindowOverlaps(Di, g[v], world, controlGoal.get_disturbance())){
					Di.bf.pose=b2Mul(invmul, Di.bf.pose); //DISTURBANCE FORWARD PROP
					return Di;
				}
			}
			//check if Di was eliminated
			return controlGoal.get_disturbance();
		} 
		else if ( CVDi.isValid()){ //if the configurator made a mental note to remmeber a disturbance
			// && g[v].isTurning()==isTurning(dir)
			CVDi.bf.pose= b2Mul(invmul, CVDi.bf.pose); //DISTURBANCE BACK-AND-ACROSS PROP
			return CVDi;
		} 
		else  if (v==MOVING_VERTEX){
			return g[v].Di;
		}
	}
	Disturbance Dn= g[v].Dn;
	Dn.bf.pose=b2Mul(invmul, Dn.bf.pose);
	return Dn;
}


// void B2BConfigurator::transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src){
// 	Task temp(controlGoal.get_disturbance(), DEFAULT, transitionSystem[v].endPose); //reflex to disturbance
// 	srand(unsigned(time(NULL)));
// 	auto oe=gt::outEdges(transitionSystem, v, d);
// 	if (( !currentTask.get_change() ||!oe.empty()) && (iteration>1)){
// 		std::pair<bool, edgeDescriptor> ve=gt::visitedEdge(oe, transitionSystem, currentVertex);
// 		transitionSystem[v].options=partiallyExplorativeOptions(ve);
// 	}
// 	else if (transitionSystem[v].outcome==simResult::successful) { //will only enter if successful
// 		if (d== LEFT || d == RIGHT){
// 			auto defaultVisited=gt::visitedEdge(gt::outEdges(transitionSystem, v, DEFAULT), transitionSystem, currentVertex);
// 			if (!defaultVisited.first){ //used to be just the inside of this statement
// 				transitionSystem[v].options = {DEFAULT};
// 				if ((src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.get_disturbance().pose().p)<0) ){
// 					transitionSystem[v].options.push_back(d);
// 				}
// 			}
// 			else if (transitionSystem[defaultVisited.second.m_target].outcome==simResult::crashed){
// 				transitionSystem[v].options.push_back(d);
// 			}
// 		}
// 		else {
// 			 if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
// 				transitionSystem[v].options.push_back(temp.get_direction());
// 				transitionSystem[v].options.push_back(getOppositeDirection(temp.get_direction()).second);
// 				transitionSystem[v].options.push_back(DEFAULT);
// 			}
// 			else{
// 				transitionSystem[v].options= transitionInHindsight(v, [&](vertexDescriptor v){ //LAMBDA FUNCTION!
// 					std::vector <Direction> result={DEFAULT};
// 					return result;
// 				});
// 			}

// 		}
// 	}
// }

// void B2BConfigurator::removeExploredTransitions( vertexDescriptor v){
// 	std::vector<Direction>options=getExploredDirections(v, transitionSystem[v].options);
// 	for (Direction d:options){
// 		erase_from_vector(transitionSystem[v].options, d);
// 	}
// }

std::vector<vertexDescriptor> B2BConfigurator::explorer(vertexDescriptor v, TransitionSystem& g, b2World & w){
	if (transitionSystem.m_vertices.size()==0){
		throw "no dummy vertex!";
	}
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	Direction direction=currentTask.get_direction();
	std::vector <vertexDescriptor> priorityQueue = {v}, evaluationQueue, plan_prov=m_plan;
	std::set <vertexDescriptor> closed;
	Task t=currentTask;
	b2Transform start= b2Transform_zero, shift=b2Transform_zero, shift_start=shift;
	EndedResult er;
	do{
		v=bestNext;
		vertexDescriptor startRecycle=v;
		bool wasClosed =closeVertex(closed, v);
		priorityQueue.erase(priorityQueue.begin());
		er = controlGoal.checkEnded(g[v], t.get_direction());
		applyTransitionMatrix(v, direction, er.ended, v, plan_prov);
		EvaluationQueueManager eqm;
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0_exp=v;
			std::vector <Direction> options=g[v0_exp].options;
			while (!options.empty()){
				options.erase(options.begin());
				v0=v0_exp; //node being expanded
				v1 =v0; //frontier
				do {
				std::pair<State, Edge> sk=simulation_setup(w, t, v0, shift, start, g[v0].options);
				simResult sim=simulate(t, w, v0);
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.it_observed=iteration;
				er  = estimateCost(sk.first, g[v0].endPose, sk.first.direction,controlGoal);
				StateDifference sd;
				VertexMatch match=findMatch(sk.first, t.get_direction(), StateMatcher::MATCH_TYPE::ABSTRACT, &sd);		//, closest_match
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher->match_equal(match.first,StateMatcher::MATCH_TYPE::ABSTRACT)){
					g[v0].options.erase(g[v0].options.begin());
					edge=setup_match_edge(match, v0, v1, sk.second, t.get_direction(), false);
					if (currentTask.is_over()){
						std::vector <vertexDescriptor> task_vs= task_vertices(v1);
						vertexDescriptor task_start= task_vs[0];
						startRecycle=getRecyclingStart(v, v1, task_start);
						if (plan_prov.empty()){
							recycle_plan(startRecycle, v0, task_start, match.first, shift_start, sk.first.start, edge, plan_prov, t.get_direction());
						}
						if (m_plan.empty() && g[task_start].options.empty() && g[v].options.empty()){
							if (startRecycle!=v){
								task_vs.push_back(startRecycle);
							}
							shift_states(g, task_vs, shift_start);
						}
					}
				}
				else{
					edge= add_vertex_now(v0, v1,sk.first.Di, sk.second); //addVertex
					abandonPlan(plan_prov, v0, v1);
					shift=b2Transform_zero;
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, iteration);
					//adjustProbability(edge.first); //new_edge to allow to adjust prob if the sim state has been previously ecountered and split
				}
				applyTransitionMatrix(v1, t.get_direction(), er.ended, v0, plan_prov);
				g[v1].phi=evaluationFunction(er, v1, plan_prov);
				propagateD(v1, v0, &closed); //if v0 is a dummy vertex it propagates the disturbance
				addOptionsInHindsight(v, v0, v1, clearvoyance); //if default move fails, adds another default option to avoid this disturbance
				v0_exp=v0;
				options=g[v0_exp].options;
				v0=v1;
				eqm.addToEvaluationQueue(evaluationQueue, v1, transitionSystem, v);
			}while(t.get_direction() !=DEFAULT & int(g[v0].options.size())!=0);
		}
	}
	backtrack(evaluationQueue, priorityQueue, closed, plan_prov, v, startRecycle);
	bestNext=priorityQueue[0];
	reassign_direction(bestNext, direction);
//}while(g[bestNext].options.size()>0 && !er.ended);
}while(!priorityQueue.empty() && !er.ended);
clearvoyance.reset();
return plan_prov;
}

simResult B2BConfigurator::simulate(Task  t, b2World & w, vertexDescriptor v){ //State& state, State src,
	simResult result;
	float remaining=remainingSimulationTime();
	Disturbance focus=controlGoal.get_disturbance();
	if (Disturbance maybeFocus=clearvoyance.query(v); maybeFocus.isValid()){//&& isTurning(t.get_direction())==transitionSystem[v].isTurning()
		maybeFocus.set_affordance(PURSUE);
		focus=maybeFocus;
		maybeFocus.bf.attention=true;
		worldBuilder->makeBody(w, maybeFocus.bf); //add hindsight disturbance to the world even if it doesn't overlap with the task scope
		clearvoyance.pop(v);
	}
	Robot robot=makeRobot(w, t.getStart(), focus);
	worldBuilder->add_body_count();
	simulatedTasks++;
	result =t.bumping_that(w, iteration, robot.body(), remaining); //default start from 0
	//approximate angle to avoid rounding errors
	//b2Transform travelTransform=b2MulT(result.endPose, t.start);
	result.endPose.q.Set(approximate_angle(result.endPose.q.GetAngle(), t.get_direction(), result.resultCode));
	return result;
}

Robot B2BConfigurator::makeRobot( b2World & world, const b2Transform& start, const Disturbance & focus){
	Robot robot=Configurator::makeRobot(world, start);
	b2AABB sensor_aabb=worldBuilder->makeRobotSensor(robot.body(), focus);
	return robot;

}


void B2BConfigurator::addOptionsInHindsight(vertexDescriptor v, vertexDescriptor v0, vertexDescriptor v1, ClearVoyance & clearvoyance){
	if (!transitionSystem[v1].isTurning() && transitionSystem[v0].isTurning() && transitionSystem[v1].outcome==simResult::crashed){ //
		transitionSystem[v].options.push_back(DEFAULT);
		clearvoyance.add(v, transitionSystem[v1].Dn);
		clearvoyance.add(v0, transitionSystem[v1].Dn);
	}

}

bool B2BConfigurator::ClearVoyance::add(vertexDescriptor v, const Disturbance &d){
	bool result=false;
	if (d.getAffIndex()==NONE){
		return result;
	}
	auto vIt=std::find_if(lookaheads.begin(), lookaheads.end(), [&](const DisturbanceLookahead & dl){return dl.source==v;});
	if (vIt==lookaheads.end()){
		lookaheads.emplace_back(ClearVoyance::DisturbanceLookahead(v, d));
		result=true;
	}
	else if (std::find_if(vIt->disturbances.begin(), vIt->disturbances.end(), [&](const Disturbance & dd){return dd==d;})!=vIt->disturbances.end()){
		vIt->disturbances.push_back(d); //update disturbance
		result=true;
	}
	return result;

}

Disturbance B2BConfigurator::ClearVoyance::query(vertexDescriptor v){
	auto vIt=std::find_if(lookaheads.begin(), lookaheads.end(), [&](const DisturbanceLookahead & dl){return dl.source==v;});
	if (vIt!=lookaheads.end()){
		if (!vIt->disturbances.empty()){
			return *(vIt->disturbances.begin()); //return the last disturbance
		}
	}
	return Disturbance();
}

void B2BConfigurator::ClearVoyance::pop(vertexDescriptor v){
	auto vIt=std::find_if(lookaheads.begin(), lookaheads.end(), [&](const DisturbanceLookahead & dl){return dl.source==v;});
	if (vIt!=lookaheads.end()){
		if (!vIt->disturbances.empty()){
			vIt->disturbances.erase(vIt->disturbances.begin());
		}
		else{
			lookaheads.erase(vIt); //erase the lookahead if no disturbances leftS
		}
	}
}

