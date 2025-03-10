#include "planner.h"

void Planner::path2add2(std::vector<std::vector<vertexDescriptor>>::reverse_iterator & path, const std::vector <vertexDescriptor> & add, std::vector<std::vector<vertexDescriptor>> &paths, TransitionSystem &g){
    	std::pair<edgeDescriptor, bool> edge(edgeDescriptor(), false);
		std::vector<vertexDescriptor>::reverse_iterator pend=(path->rbegin());
		while (!edge.second){//|| ((*(pend.base()-1)!=goal &goal!=TransitionSystem::null_vertex())&!controlGoal.checkEnded(g[*(pend.base()-1)]).ended)
			vertexDescriptor end=*(pend.base()-1); //equivalent to path.end()
			edge= boost::edge(end,add[0], g);
			if (!add.empty()&!edge.second & path!=paths.rend()){ //if this path does not have an edge and there are 
													//other possible paths, go to previous paths
				if (pend.base()-1!=(path->begin())){ //if the current vertex is not the root of the path
					pend++; //go back a step
				}
				else{
					path++; //go back a previously explored path
					pend=(*path).rbegin(); 
				}
			}
			else if (edge.second & pend.base()!=path->rbegin().base()){  //if there is an edge with the end of current path
				bool found=0;
				for (auto _p=paths.rbegin(); _p!=paths.rend(); _p++ ){ // see if theres a path with this beginning and end
					if (std::vector <vertexDescriptor>(path->begin(), pend.base())==*_p){
						path=_p; //switch to this path
						found=1;
					}
				}
				if (!found){
					//create new empty path
					paths.emplace_back(std::vector <vertexDescriptor>(path->begin(), pend.base()));
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


std::vector <vertexDescriptor> Planner::best_path(const std::vector<std::vector<vertexDescriptor>>& paths, const vertexDescriptor& goal, const vertexDescriptor& cv, const bool & change, const TransitionSystem& g){
    std::vector <vertexDescriptor> plan;
    float final_phi=10000;
	for (std::vector<vertexDescriptor> p: paths){
		vertexDescriptor end_plan= *(p.rbegin().base()-1);
		//LAMBDA
		auto skip_first= [](const std::vector<vertexDescriptor> &_plan, const vertexDescriptor & _cv, const TransitionSystem & _g, const bool & _change){
        bool empty_xor_currentv= (_plan.size()==1 ^ _plan[0]!=_cv);
        if (empty_xor_currentv && _change){ //&& _plan[0]==_cv
				return std::vector(_plan.begin()+0, _plan.end());
			}
			else{
				return std::vector((_plan.begin()+1), _plan.end());
			}
		}; //END LAMBDA 
		if (end_plan==goal){
			plan=skip_first(p, cv, g, change);
			break;
		}
		else if (g[end_plan].phi<final_phi){
			plan=skip_first(p, cv, g, change);
			final_phi=g[end_plan].phi;
		}
	}
    return plan;

}
