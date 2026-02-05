#include "planner.h"

float evaluationFunction(EndedResult er,  const vertexDescriptor& v, std::vector<vertexDescriptor>& p){ 
	float result=(abs(er.estimatedCost)+abs(er.cost))/2;
//	if (auto it=check_vector_for(p, v); it!=p.end()){
	if (std::find(p.cbegin(), p.cend(), v)!=p.end()){
		result-=0.1;
	}
	return result; //normalised to 1
}


EndedResult estimateCost(const State &state, b2Transform start, Direction d, Task &_goal){
	EndedResult er = _goal.checkEnded(state);
	Task t(state.Dn, d, start);
	er.cost += t.checkEnded(state.endPose).estimatedCost;
	return er;
}

void HorizonStarPlanner::path2add2(std::vector<std::vector<vertexDescriptor>>::reverse_iterator & path, const std::vector <vertexDescriptor> & add, std::vector<std::vector<vertexDescriptor>> &paths, TransitionSystem &g){
    	std::pair<edgeDescriptor, bool> edge(edgeDescriptor(), false);
		std::vector<vertexDescriptor>::reverse_iterator path_end_rit=(path->rbegin()); //reverse iterator to end of path
		while (!edge.second){
			vertexDescriptor end=*(path_end_rit.base()-1); //equivalent to path.end()
			edge= boost::edge(end,add[0], g);
			if (!add.empty()&&!edge.second && path!=paths.rend()){ //if this path does not have an edge and there are 
													//other possible paths, go to previous paths
				if (path_end_rit.base()-1!=(path->begin())){ //if the current vertex is not the root of the path
					path_end_rit++; //go back a step
				}
				else{
					path++; //go back a previously explored path
					path_end_rit=(*path).rbegin(); //reset the path end
				}
			}
			else if (edge.second && path_end_rit.base()!=path->rbegin().base()){  //if there is an edge with the end of current path
				bool found=0; 
				// for (auto _p=paths.rbegin(); _p!=paths.rend(); _p++ ){ // see if theres a path with this beginning and end
				// 	if (std::vector <vertexDescriptor>(path->begin(), path_end_rit.base())==*_p){
				// 		path=_p; //switch to this path
				// 		found=1;
				// 	}
				// }
				struct HasStartStop{
					vertexDescriptor start=TransitionSystem::null_vertex(), stop=TransitionSystem::null_vertex();
					HasStartStop(vertexDescriptor _start, vertexDescriptor _stop):start(_start), stop(_stop){}
					
					bool operator()(const std::vector<vertexDescriptor>& v){
						if (v.empty()){return false;}
						return *v.cbegin()==start && *(v.cend()-1)==stop;
					}
				};

				auto _p=std::find_if(paths.rend(), paths.rbegin(), HasStartStop(*path->begin(), *path_end_rit.base()));
				if (_p!=paths.rbegin()){path=_p;}
				else{
					//create new empty path
					paths.emplace_back(std::vector <vertexDescriptor>(path->begin(), path_end_rit.base()));
					path=paths.rbegin();				
				}
				break;
			}
			if ( path==paths.rend()) { //if there are no other paths
				paths.push_back(std::vector<vertexDescriptor>()); //make a new one
				path=paths.rbegin();
				break;
			}
		}
}


std::vector <vertexDescriptor> HorizonStarPlanner::best_path(const std::vector<std::vector<vertexDescriptor>>& paths, vertexDescriptor goal, vertexDescriptor cv, bool  change, const TransitionSystem& g){
    std::vector <vertexDescriptor> result;
    float final_phi=10000;
		//LAMBDA
	auto skip_first= [](const std::vector<vertexDescriptor> &_plan, const vertexDescriptor & _cv, const TransitionSystem & _g, const bool & _change){
	bool empty_xor_currentv= (_plan.size()==1 ^ (_plan[0]!=_cv &&_plan[0]!=MOVING_VERTEX));
	if (empty_xor_currentv && _change){ //&& _plan[0]==_cv
			return std::vector(_plan.begin()+0, _plan.end());
		}
		else{
			return std::vector((_plan.begin()+1), _plan.end());
		}
	}; //END LAMBDA 

	for (std::vector<vertexDescriptor> p: paths){
		vertexDescriptor end_plan= *(p.rbegin().base()-1);
		if (end_plan==goal){
			result=skip_first(p, cv, g, change);
			break;
		}
		else if (g[end_plan].phi<final_phi){
			result=skip_first(p, cv, g, change);
			final_phi=g[end_plan].phi;
		}
	}
    return result;
}

std::vector <Frontier> frontierVertices(vertexDescriptor v, TransitionSystem& g, ExecutionInfo & info){
	std::vector <Frontier> result;
	std::pair<edgeDescriptor, bool> ep=boost::edge(MOVING_VERTEX, v, g); 
	vertexDescriptor v0=v, v1=v, v0_exp;
	//do{
		if ((info.overarchingGoal().get_disturbance().getPosition()-g[v].endPose.p).Length() >= DISTANCE_ERROR_TOLERANCE){
			auto es=boost::out_edges(v, g);
			for (auto ei=es.first; ei!=es.second; ei++){
			std::vector <vertexDescriptor>connecting;
			auto ei2=ei, ei3=ei;
			auto es2=boost::out_edges((*ei).m_target, g);
			auto es3=es2;
			std::vector <vertexDescriptor>connecting2;
			NotSelfEdge not_self_edge(&g);
			do {
				//g[(*ei3).m_target].visited()
				if ((g[(*ei3).m_target].visited()|| info.been())&& g[(*ei3)].it_observed>=0 &&(not_self_edge(*ei3) || g[*ei3].overrideZeroSteps)){ //(*ei3).m_source!=(*ei3).m_target
					if (g[*ei3].overrideZeroSteps){
						g[*ei3].overrideZeroSteps=false;
					}
					if (!g[(*ei3).m_target].visited()){
						EndedResult er = estimateCost(g[(*ei3).m_target], g[(*ei3).m_source].endPose, g[(*ei3).m_target].direction,info.overarchingGoal());
						std::vector<vertexDescriptor>_plan=info.plan();
						g[(*ei3).m_target].phi=evaluationFunction(er, (*ei3).m_target, _plan);
					}
					if (g[(*ei3).m_target].direction==DEFAULT){ // The depth-first search portion of iterative deepening will stop when it reaches a DEFAULT task
						Frontier f;
						f.frontier= (*ei3).m_target;
						f.connecting=connecting2;
						result.push_back(f);
						if (ei3!=ei){
							ei3++;
						}
						else{
							connecting.clear();
							break;
						}
					}
					else if (ei3==ei){
						connecting.push_back((*ei3).m_target);
						connecting2=connecting;
						es3=boost::out_edges((*ei3).m_target,g);
						ei3=es3.first;
						ei2=ei3;
						es2=es3;
					}
					else if (ei2!=ei){
						connecting2.push_back((*ei3).m_target);
						es3=boost::out_edges((*ei3).m_target,g);
						ei3=es3.first;
						ei2++;
					}
				}
				else if (ei3!=ei){
					ei3++;
				}
				else { //not sure if this is right just added
					break;
				}
				if(ei3==es3.second){
					if (ei3!=ei2 && ei2!=ei){
						if (ei2!=es2.second){
							ei2++;
							ei3=ei2;
							es3=es2;						
						}
					}					
				}

				//printf("is stuck, ei3=%i ->%i\n", (*ei).m_source, (*ei).m_target);
			}while (ei3!=es3.second);
		}
	}

	return result;
}

void HorizonStarPlanner::addToPriorityQueue(const Frontier& f, std::vector<Frontier>& queue, TransitionSystem &g,const  std::set<vertexDescriptor>&closed, vertexDescriptor goal){
	if (auto it_q=std::find(queue.cbegin(), queue.cend(), f); it_q!=queue.end()){
		return;
	}
	if (g[*it_q].options.empty()){
		closed.insert(*it_q);
		return;
	}
	for (auto i =queue.begin(); i!=queue.end(); i++){
		auto it=std::find(closed.begin(), closed.end(), f.frontier);
		if (it!=closed.end()){
			return;
		}
		if (g[f.frontier].phi <abs(g[(*i).frontier].phi)){
			queue.insert(i, f);
			return;
		}
	}
	queue.push_back(f);
}

std::vector <vertexDescriptor> HorizonStarPlanner::plan( TransitionSystem& g, vertexDescriptor src, ExecutionInfo& info, bool *finished){
	std::vector<std::vector<vertexDescriptor>> paths;
	std::set<vertexDescriptor> closed;
	paths.push_back(std::vector<vertexDescriptor>()={src});
	std::vector <Frontier> frontier_v;
	bool _finished=false;
	std::vector <Frontier> priorityQueue={Frontier(src, std::vector<vertexDescriptor>())};

	int no_out=0;
	std::vector <vertexDescriptor> add;
	std::vector<std::vector<vertexDescriptor>>::reverse_iterator path= paths.rbegin();
	vertexDescriptor path_end=src, goal=info.goalVertex();
	auto start_time=std::chrono::high_resolution_clock::now();
	do{
		frontier_v=frontierVertices(src, g, info); // get next default tasks (plus non-default connecting tasks)
		closed.emplace(src);
		priorityQueue.erase(priorityQueue.begin());
		for (Frontier f: frontier_v){ //add to priority queue
			//planPriority(g, f.first);
			addToPriorityQueue(f, priorityQueue, g, closed);
		}
		if (!priorityQueue.empty()){
			src=priorityQueue.begin()->frontier; //lowest phi vertex
			add=std::vector <vertexDescriptor>(priorityQueue.begin()->connecting.begin(), priorityQueue.begin()->connecting.end());//lowest phi frontier
			add.push_back(src);
			path2add2(path, add, paths, g); //find path to add frontier (add) to
			for (vertexDescriptor c:add){
				g[c].label=VERTEX_LABEL::UNLABELED;
				path->push_back(c);	
				path_end=c;			
			}
		}
		_finished=info.overarchingGoal().checkEnded(g[path_end].endPose, UNDEFINED, true).ended;
		if (NULL!=finished){
			*finished=_finished;
		}
		if (_finished){
			goal=path_end;
		}
	}while(!priorityQueue.empty() && (path_end!=goal && !(_finished)));
	if (_finished || info.overarchingGoal().getAffIndex()==NONE){
		return best_path(paths, goal, info.currentVertex(), info.currentTask().get_change(), g);
	}
	if (!_finished){
		//return best_path(paths, TransitionSystem::null_vertex(), info.currentVertex(), info.currentTask().get_change(), g);
	}
	return std::vector<vertexDescriptor>();
}
