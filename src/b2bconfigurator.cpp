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

std::vector<Direction> B2BConfigurator::partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve){
	std::vector <Direction> result;
	if (ve.first){
		if(transitionSystem[ve.second.m_target].visited()){
			if (transitionSystem[ve.second.m_target].outcome!=simResult::crashed){
				return {transitionSystem[ve.second.m_target].direction};
			}
			else if (transitionSystem[ve.second.m_target].outcome==simResult::crashed){
				// ExecutionInfo info=package_info();
				// std::vector <Frontier> frontiers=frontierVertices(ve.second.m_source, transitionSystem, info);
				// if (frontiers.size()<2){ //only default explored
				// 	result={DEFAULT, LEFT, RIGHT};
				// 	erase_from_vector(result, transitionSystem[ve.second.m_target].direction);
				// 	return result;
				// }
				// else if (frontiers.size()<4){ //left right explored
                //     auto fLeft= std::find_if(frontiers.begin(), frontiers.end(), FrontierCrashed(transitionSystem, LEFT));
				// 	auto fRight= std::find_if(frontiers.begin(), frontiers.end(), FrontierCrashed(transitionSystem, RIGHT));
                //     if (fLeft!=frontiers.end()) result.push_back(DEFAULT);
                //     if (fRight!=frontiers.end()) result.push_back(DEFAULT);
				// }
				transitionInHindsight(ve.second.m_source, [&](vertexDescriptor v){ //LAMBDA FUNCTION! 
					std::vector <Direction> result={DEFAULT, LEFT, RIGHT};
					erase_from_vector(result, transitionSystem[v].direction);
					return result;
				});
			}
		}
}
return result;
}

std::vector <vertexDescriptor> B2BConfigurator::splitTask(vertexDescriptor v, Direction d, vertexDescriptor src){
    std::vector <vertexDescriptor> split={v};
	auto first_edge=boost::edge(src, v, transitionSystem); //assumes exists
	if (transitionSystem[v].isTurning()){ //d
		if ((src==MOVING_VERTEX || src==DUMMY )&& transitionSystem[v].outcome==simResult::crashed){
			split.emplace(split.begin(), src);
		}
	}		
    return split;
}

Disturbance B2BConfigurator::getDisturbance(TransitionSystem&g,vertexDescriptor v, b2World & world, const Direction& dir, const b2Transform& start){
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=inEdges(v);
		std::vector <edgeDescriptor> out=gt::outEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);
			if (visited.first ||out.empty()){
				if (g[v].Di.isValid() && g[v].Di.getAffIndex()==AVOID && g[v].direction!=dir){
					Task task(g[v].Di, DEFAULT, g[v].endPose, true);
					Robot robot(&world);
					robot.body()->SetTransform(task.getStart().p, task.getStart().q.GetAngle());
					b2AABB box =worldBuilder.makeRobotSensor(robot.body(), controlGoal.get_disturbance_ptr());
					b2Fixture *sensor =GetSensor(robot.body());
					bool overlap=overlaps(robot.body(), &g[v].Di) && sensor;
					world_cleanup(world);
					if (overlap){
						Disturbance Di= g[v].Di;
						Di.bf.pose+= start-g[v].endPose;
						return Di;
					}
				}
				//check if Di was eliminated 
				return controlGoal.get_disturbance();
			}
			else if (v==MOVING_VERTEX){
				return g[v].Di;
			}
	}
	Disturbance Dn= g[v].Dn;
	Dn.bf.pose+= start-g[v].endPose;
	return Dn;
}


void B2BConfigurator::transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src){
	Task temp(controlGoal.get_disturbance(), DEFAULT, transitionSystem[v].endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	auto oe=gt::outEdges(transitionSystem, v, d);
	if (( !currentTask.get_change() ||!oe.empty()) && (iteration>1)){
		std::pair<bool, edgeDescriptor> ve=gt::visitedEdge(oe, transitionSystem, currentVertex);
		transitionSystem[v].options=partiallyExplorativeOptions(ve);
	}
	else if (transitionSystem[v].outcome==simResult::successful) { //will only enter if successful
		if (d== LEFT || d == RIGHT){
			auto defaultVisited=gt::visitedEdge(gt::outEdges(transitionSystem, v, DEFAULT), transitionSystem, currentVertex);
			if (!defaultVisited.first){ //used to be just the inside of this statement
				transitionSystem[v].options = {DEFAULT};
				if ((src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.get_disturbance().pose().p)<0) ){
					transitionSystem[v].options.push_back(d);
				}
			}
			else if (transitionSystem[defaultVisited.second.m_target].outcome==simResult::crashed){
				transitionSystem[v].options.push_back(d);
			}
		}
		else {
			 if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
				transitionSystem[v].options.push_back(temp.get_direction());
				transitionSystem[v].options.push_back(getOppositeDirection(temp.get_direction()).second);
				transitionSystem[v].options.push_back(DEFAULT);
			}
			else{
				transitionInHindsight(v, [&](vertexDescriptor v){ //LAMBDA FUNCTION! 
					std::vector <Direction> result={DEFAULT}; 
					return result;
				});
				//transitionSystem[v].options={DEFAULT};
			}

		}

	}
}