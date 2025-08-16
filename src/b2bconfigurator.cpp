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
				ExecutionInfo info=package_info();
				std::vector <Frontier> frontiers=frontierVertices(ve.second.m_source, transitionSystem, info);
				if (frontiers.size()<2){ //only default explored
					result={DEFAULT, LEFT, RIGHT};
					erase_from_vector(result, transitionSystem[ve.second.m_target].direction);
					return result;
				}
				else if (frontiers.size()<4){ //left right explored
                    auto fLeft= std::find_if(frontiers.begin(), frontiers.end(), FrontierCrashed(transitionSystem, LEFT));
					auto fRight= std::find_if(frontiers.begin(), frontiers.end(), FrontierCrashed(transitionSystem, RIGHT));
                    if (fLeft!=frontiers.end()) result.push_back(DEFAULT);
                    if (fRight!=frontiers.end()) result.push_back(DEFAULT);
				}
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