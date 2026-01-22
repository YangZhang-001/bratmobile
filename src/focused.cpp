#include "focused.h"


std::pair <edgeDescriptor, bool> FocusedConfigurator::add_vertex_now(const vertexDescriptor & src, vertexDescriptor &v1, Disturbance Di,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, edge, topDown);
	if (!transitionSystem[v1].filled){
		transitionSystem[v1].Di= Di;
	}
	return result;
}

std::pair <edgeDescriptor, bool> FocusedConfigurator::add_vertex_retro(vertexDescriptor & src, vertexDescriptor &v1,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, edge, topDown);
	transitionSystem[v1].Di= transitionSystem[src].Di;
	transitionSystem[v1].Dn=transitionSystem[src].Dn;
	return result;
}


void FocusedConfigurator::resetPhi(){
	auto vs=boost::vertices(transitionSystem);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		transitionSystem[*vi].resetVisited();
		transitionSystem[*vi].options.clear();
	}
}

float FocusedConfigurator::customSimulationStep(vertexDescriptor v){
	if (v!=TransitionSystem::null_vertex()){
		return std::max(simulationStep, transitionSystem[v].distance()/2);
	}
	return simulationStep;
}



Disturbance FocusedConfigurator::getDisturbance(TransitionSystem&g,vertexDescriptor v, b2World & world, const Direction& dir, const b2Transform& start){
	b2Transform invmul=b2help::InvMul(start,g[v].endPose);
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=inEdges(v);
		std::vector <edgeDescriptor> out=gt::outEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);
		if (visited.first ||out.empty()){
			if (g[v].Di.isValid() && g[v].Di.getAffIndex()==AVOID && (g[v].direction!=dir || (g[v].isTurning() && isTurning(dir)))){ //if Di is valid and not the same direction as the vertex
				Disturbance Di= g[v].Di;
				if (std::pair <bool, edgeDescriptor> visitedDefault=gt::visitedEdge(gt::outEdges(g, v, DEFAULT), g, v); visitedDefault.first && (g[v].isTurning() && isTurning(dir))){
					if (visitedDefault.first && g[visitedDefault.second.m_target].outcome==simResult::crashed){
						Di= g[visitedDefault.second.m_target].Di;
					}
				} //if the vertex has been visited in the default direction
				Task task(Di, DEFAULT, g[v].endPose, true);
				Robot robot(&world);
				robot.body()->SetTransform(task.getStart().p, task.getStart().q.GetAngle());
				b2AABB box =worldBuilder->makeRobotSensor(robot.body(), controlGoal.get_disturbance());
				b2Fixture *sensor =GetSensor(robot.body());
				bool overlap=overlaps(robot.body(), &Di) && sensor;
				world_cleanup(world);
				if (overlap){
					Di.bf.pose=b2Mul(invmul, Di.bf.pose);
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
	Dn.bf.pose=b2Mul(invmul, Dn.bf.pose);
	return Dn;
	//return controlGoal.disturbance;
}



std::vector<vertexDescriptor> FocusedConfigurator::explorer(vertexDescriptor v, TransitionSystem& g, b2World & w){
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
		er = controlGoal.checkEnded(g[v], t.get_direction(), true); //check ended with relax
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
				simResult sim=simulate(t, w); //sk.first, g[v0], 
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.it_observed=iteration;
				er  = estimateCost(sk.first, g[v0].endPose, sk.first.direction,controlGoal);
				StateDifference sd;
				VertexMatch match=findMatch(sk.first, t.get_direction(), desiredMatch(), &sd);		//, closest_match	
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher.match_equal(match.first,desiredMatch())){
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
					edge= add_vertex_now(v0, v1,sk.first.Di, sk.second);
					abandonPlan(plan_prov, v0, v1);
					shift=b2Transform_zero;
					if (iteration>1){
						if(sk.first.outcome==simResult::crashed && v0==MOVING_VERTEX && sk.first.direction==currentTask.get_direction()){
							debug::print_pose(sk.first.Dn.bf.pose, "crash site");
						}
					}
				}
				if(edge.second){ //edge was added
					gt::set(edge.first, sk, g, v1==currentVertex, iteration);
					//adjustProbability(edge.first); //new_edge to allow to adjust prob if the sim state has been previously ecountered and split
				}
				applyTransitionMatrix(v1, t.get_direction(), er.ended, v0, plan_prov);
				g[v1].phi=evaluationFunction(er, v1, plan_prov);
				propagateD(v1, v0, &closed); //if v0 is a dummy vertex it propagates the disturbance
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
}while(g[bestNext].options.size()>0 && !er.ended);
return plan_prov;
}

std::vector <vertexDescriptor> FocusedConfigurator::splitTask( vertexDescriptor v,  Direction d, vertexDescriptor src){
	std::vector <vertexDescriptor> split={v};
	auto first_edge=boost::edge(src, v, transitionSystem); //assumes exists
	//if (gt::check_edge_direction(first_edge, transitionSystem, RIGHT)|| gt::check_edge_direction(first_edge, transitionSystem, LEFT)){ //d
	if (transitionSystem[v].isTurning()){ //d
		if ((src==MOVING_VERTEX || src==DUMMY )&& transitionSystem[v].outcome==simResult::crashed){
			split.emplace(split.begin(), src);
		}
		return split;
	}
	if (transitionSystem[v].outcome != simResult::crashed){
		return split;
	}
	float _customStep= customSimulationStep(v);
	auto ie=inEdges(src);
	auto sameIterationEdgeIt=check_vector_for(ie, SameIteration(transitionSystem, iteration));
	if (!transitionSystem[src].isTurning()&& (!ie.empty()|| src==MOVING_VERTEX)){ //! //&& sameIterationEdgeIt!=ie.end()
		transitionSystem[src].outcome=simResult::safeForNow;
		split.insert(split.begin(), src);
	}
	vertexDescriptor v1=v;
	float nNodes = transitionSystem[v].distance()/_customStep, og_phi=transitionSystem[v].phi;
	b2Transform endPose = transitionSystem[v].endPose;
	Task::Action a;
	a.init(d);
	b2Transform deltaTransform=b2Transform_zero;
	deltaTransform.p.x=_customStep;
	while(nNodes>1){
		State s_tmp=State(transitionSystem[v]);
		if(nNodes >1){
			s_tmp.endPose=b2Mul(transitionSystem[v].start, deltaTransform);
			VertexMatch match=hardMatch(s_tmp, d);
			if (match.first!=StateMatcher::_TRUE){
				first_edge=addEdgeRetrospectively(v, v1, s_tmp, first_edge, d, a.getLinearSpeed());
			}
			else{
				v1=match.second;
			}
			split.push_back(v1);
			nNodes--;
		}
		if (nNodes<=1){
			s_tmp.endPose=endPose;
			VertexMatch match=hardMatch(s_tmp, d);
			if (match.first!=StateMatcher::_TRUE || match.second==v){
				transitionSystem[v1].endPose = endPose;
				transitionSystem[first_edge.first].step= gt::distanceToSimStep(transitionSystem[v1].distance(), a.getLinearSpeed());	
				transitionSystem[v1].outcome=simResult::crashed;	
				transitionSystem[v1].phi=og_phi;
			}
		}
		v=v1;
	}
	return split;
}


void FocusedConfigurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src, vertexDescriptor startRecycle){
	for (vertexDescriptor v:evaluation_q){
		std::pair<bool, edgeDescriptor> ep(false, edgeDescriptor());
		std::vector <vertexDescriptor> split = task_vertices(v, &ep); 
		Direction direction= transitionSystem[ep.second.m_target].direction;
		if (split.size()<2){
			split =splitTask(v, DEFAULT, ep.second.m_source);
			if (split.size()>1){
				if (split[split.size()-1]!=v){
					split.erase(split.begin()+split.size()-1); //hotfix
				}
			}
		}
		correctQueue(split, module_src, startRecycle, plan_prov.size());
		for (int i=split.size()-1; i>=0; i--){ //
			vertexDescriptor split_v=split[i], src=TransitionSystem::null_vertex();
			if (i<1){
				auto likelyEdge=gt::getMostLikely(transitionSystem, inEdges(split_v), iteration);
				if (likelyEdge.first){
					src=likelyEdge.second.m_source;
				}
				else{
					src=split_v;
				}
			}
			else{
				src=split[i-1];
			}
			EndedResult local_er=estimateCost(transitionSystem[split_v],transitionSystem[split_v].start,direction, controlGoal);
			transitionSystem[split_v].phi=evaluationFunction(local_er, split_v, plan_prov);
			applyTransitionMatrix(split_v, direction, local_er.ended,src, plan_prov);
			addToPriorityQueue(split_v, priority_q, closed);
			src=split_v;
		}
	}
	evaluation_q.clear();
}

bool FocusedConfigurator::canPropagate(vertexDescriptor v){
	return transitionSystem[v].direction==STOP;
}


bool FocusedConfigurator::canReassignOutcome(vertexDescriptor v){
	return v==currentVertex;
}

bool FocusedConfigurator::propagateD(vertexDescriptor v1, vertexDescriptor v0, std::set <vertexDescriptor>*closed,StateMatcher::MATCH_TYPE match){
	if (transitionSystem[v1].outcome == simResult::successful || !boost::edge(v0, v1, transitionSystem).second){
		return false; //can't propagate
	}
	if (isTurning(transitionSystem[v1].direction)!=isTurning(transitionSystem[v0].direction)){
		return false;
	}
	bool same_Di=transitionSystem[v0].Di==transitionSystem[v1].Di;
	std::vector<vertexDescriptor> tv=task_vertices(v1);
	if ((canPropagate(v0)&& same_Di && transitionSystem[v0].Dn.getAffIndex()==NONE)){
 			transitionSystem[v0].Dn = transitionSystem[v1].Dn; //was target
 	}
	bool canReassign=canReassignOutcome(v0);
	if (canReassign){
		transitionSystem[v0].outcome=simResult::safeForNow;
	}
	return true; //theoretically could continue propagating
}

void FocusedConfigurator::visitedDirectionsPushBack( vertexDescriptor v, std::vector<Direction> & visitedDirections, Direction direction){
	for (edgeDescriptor &e: gt::outEdges(transitionSystem, v, direction)){
		if (transitionSystem[e].it_observed==iteration){ //g[e.m_target].visited()
			visitedDirections.push_back(transitionSystem[e.m_target].direction);
		}
	}

}

std::vector <Direction>  FocusedConfigurator::getExploredDirections(vertexDescriptor v, const std::vector<Direction>& directions){
	std::vector <Direction> result;
	for (Direction direction: directions){
		visitedDirectionsPushBack(v, result, direction);
	}
	return result;
}



void FocusedConfigurator::removeExploredTransitions( vertexDescriptor v){
	std::vector<Direction>options=getExploredDirections(v, transitionSystem[v].options);
	for (Direction d:options){
		erase_from_vector(transitionSystem[v].options, d);
	}
    if (preventTransition(v)){
		transitionSystem[v].options.clear();
	}
}

void FocusedConfigurator::transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src){
	Task temp(controlGoal.get_disturbance(), DEFAULT, transitionSystem[v].endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	auto oe=gt::outEdges(transitionSystem, v, d);
	std::pair<bool, edgeDescriptor> ve=gt::visitedEdge(oe, transitionSystem, currentVertex);
	if (shouldPartiallyExplore(oe, ve)){
		transitionSystem[v].options=partiallyExplorativeOptions(ve);
	}
	else if (transitionSystem[v].outcome == simResult::safeForNow){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
			transitionSystem[v].options.push_back(temp.get_direction());
			transitionSystem[v].options.push_back(getOppositeDirection(temp.get_direction()).second);
			//prioritise reflex
			if (temp.getAction().getOmega()==0){ //if the task chosen is a turning task
				if (rand()%2==0){
						transitionSystem[v].options = {LEFT, RIGHT};
				}
				else{
					transitionSystem[v].options = {RIGHT, LEFT};
				} //no idea why it doesn't work using std shuffle/swap
			}
		}
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
				transitionSystem[v].options={DEFAULT};
			}

		}

	}
}

void FocusedConfigurator::applyTransitionMatrix(vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov){
	if (!transitionSystem[v0].options.empty()){
		return;
	}
	if (controlGoal.getEndCriteria().hasEnd()){
		if (ended){
			return;
		}
	}
	else if(round(transitionSystem[v0].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		return;
	}
	if (src!=MOVING_VERTEX  && uint(src)<(transitionSystem.m_vertices.size()-1)&& v0!=MOVING_VERTEX){ //src< v size is to check that src isn't a garbage value (was giving throuble with tests)
		auto e=boost::edge(src, v0, transitionSystem); //not adding options to vertices which don't cover a distance unless they're current v
		if (e.second){
			if (transitionSystem[e.first].step==0){
				return;
			}			
		}
	}
	std::vector <vertexDescriptor> full_plan=plan_prov;
	//if (!currentTask.get_change()){ // && !full_plan.empty()
		full_plan.insert(full_plan.begin(), current_vertices.begin(), current_vertices.end());
		if (v0==MOVING_VERTEX && !currentTask.get_change()){
			full_plan.emplace(full_plan.begin(), v0);
		}
	//}
	if (auto it =check_vector_for(full_plan, v0); it!=full_plan.end() && it!=(full_plan.end()-1)){
		auto e=boost::edge(src, v0, transitionSystem);
		gt::to_task_end(e.first, transitionSystem, full_plan, it);
		if(transitionSystem[e.first.m_target].visited()){
			if (transitionSystem[e.first.m_target].outcome==simResult::crashed){
				transitionMatrix(v0, d, src);
				erase_from_vector(transitionSystem[v0].options, transitionSystem[e.first.m_target].direction);
			}
			else if (transitionSystem[e.first].it_observed<iteration){ // 
				transitionSystem[v0].options={transitionSystem[e.first.m_target].direction};
			}
		}
		else{
			transitionSystem[v0].options={transitionSystem[e.first.m_target].direction};
		}
	}
	else{
		transitionMatrix(v0, d, src);
	}
	removeExploredTransitions(v0);


}


void FocusedConfigurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, const std::set <vertexDescriptor>& closed){
	if (transitionSystem[v].outcome==simResult::crashed){
		return;
	}
	auto found=closed.find(v); 
	if(found==closed.end()){ //if not in closed
		for (auto i =queue.begin(); i!=queue.end(); i++){
			if (*i==v){
				return; //already in queue
			}
			if (transitionSystem[v].phi <abs(transitionSystem[*i].phi) ){
				queue.insert(i, v);
				return;
			}
		}
		queue.push_back(v);		
	}
}



VertexMatch FocusedConfigurator::hardMatch(State s, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd){
	VertexMatch result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex()), backup=result;
	auto vs= boost::vertices(transitionSystem);
	float prob=0, sum=10000;
	typedef std::tuple <vertexDescriptor, StateMatcher::MATCH_TYPE, float> VertexMatchTuple;
	CompareValue compareValue;
	std::set <VertexMatchTuple, CompareValue>others_set(compareValue);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=dir==Direction::UNDEFINED ||transitionSystem[v].direction==dir ||(transitionSystem[v].direction==STOP &&dir==DEFAULT &&iteration>1);
		//make state representing a whole task, this is inefficient and when i have time should be susbtituted with subgraph
		State q= transitionSystem[v];
		auto vertices=task_vertices(v);
		if ( vertices.size()>1){
			q.start=transitionSystem[vertices[0]].start;
		}			
		if (check_vector_for(vertices, currentVertex)!=vertices.end() && !currentTask.is_over()){ //v==currentVertex
			q.start=b2Transform_zero;
		}
		StateDifference sd(s, q);
		bool condition=0;
		StateMatcher::MATCH_TYPE m=StateMatcher::_FALSE;
		float sum_tmp=fabs(sd.get_sum(match_type));
		try{
			m=matcher.isMatch(sd, tracker->threshold, s.endPose.p.Length());
		}
		catch(std::exception &e){
			std::cerr<< "check tracker is set up ok! "<<e.what()<<std::endl;
		}
		condition=matcher.match_equal(m, match_type);
		if (v!=MOVING_VERTEX && (boost::in_degree(v, transitionSystem)>0 || iteration>1)  &&Tmatch ){ 
			// if (condition){
			// 	result.first= m;
			// 	result.second=v;
			// 	VertexMatchTuple to_add(v, m, sum_tmp);
			// 	others_set.emplace(to_add);
			// }
			if (sum_tmp<sum || condition){
				sum=sum_tmp;
				result.first=m;
				result.second=v;
				if (condition){
					VertexMatchTuple to_add(v, m, sum_tmp);
					others_set.emplace(to_add);

				}
				if (NULL!=_sd){
					*_sd=sd;
				}
			}
		}	
		else{
			if (sum_tmp<sum){			
				if (NULL!=_sd){
					*_sd=sd;
			}
			}

		}
	}
	// if (NULL!=other_matches){
	// 	for (VertexMatchTuple item:others_set){
	// 		other_matches->push_back(VertexMatch(std::get<1>(item), std::get<0>(item)));
	// 	}
	// }
	return result;
}

VertexMatch FocusedConfigurator::findMatch(State s, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd, vertexDescriptor src){
	if (s.start==b2Transform_zero && s.direction==currentTask.get_direction() && //s.Di==transitionSystem[currentVertex].Di && 
		!hasPlanFinished()&&	
		s.Dn.getAffIndex()==transitionSystem[currentVertex].Dn.getAffIndex()){ //if the state to be matched is the current one, return it
		return VertexMatch(StateMatcher::_TRUE, currentVertex);
	}
	return hardMatch(s, dir, match_type, _sd);
}


void FocusedConfigurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:m_plan){
		if (p==v){
       		g[v].phi-=.1;
			break;
		}
    } 
}



void FocusedConfigurator::ts_cleanup(){
	FilteredTS fts(transitionSystem, ViableEdge(&transitionSystem), Connected(&transitionSystem)); //boost::keep_all()
	TransitionSystem tmp;
	boost::copy_graph(fts, tmp);	
	transitionSystem.clear();	
	transitionSystem.swap(tmp);		
}
 
void FocusedConfigurator::shift_states(TransitionSystem & g, const std::vector<vertexDescriptor>& p, const b2Transform & shift_start){
	if (p.empty()){
		return;
	}
	for (const vertexDescriptor &v:p){
		math::MulT(shift_start, g[v]);
	}
}

vertexDescriptor FocusedConfigurator::get_explore_start(TransitionSystem & g){
	if (g.m_vertices.size()==1){
		dummy_vertex(currentVertex);
		currentTask.set_change(true);
	}
	if (!m_plan.empty() || !currentTask.is_over()){ //
		return MOVING_VERTEX;
	}
	else{
		return currentVertex;
	}
}

void FocusedConfigurator::pre_explore(){
	if (boost::out_degree(MOVING_VERTEX, transitionSystem)>0){
		boost::remove_out_edge_if(MOVING_VERTEX, is_not_v(currentVertex), transitionSystem);
	}	
	//transitionSystem[MOVING_VERTEX].Di=currentTask.get_disturbance();
	transitionSystem[MOVING_VERTEX].Di=transitionSystem[currentVertex].Di;
	transitionSystem[MOVING_VERTEX].outcome=simResult::successful;
	movingEdge=boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem).first;
//  if (currentTask.get_change()){
//  	transitionSystem[movingEdge].step=currentTask.getMotorStep();
//  }
}


void FocusedConfigurator::explore_plan(b2World&world){
    pre_explore();
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi();
	transitionSystem[MOVING_VERTEX].phi=evaluationFunction(EndedResult(), MOVING_VERTEX, m_plan);
    std::vector <vertexDescriptor> plan_tmp=explorer(src, transitionSystem, world);
    if (DEBUG){
        std::vector<vertexDescriptor> _plan=(m_plan);
        debug::graph_file(iteration, transitionSystem, controlGoal.get_disturbance(), _plan, currentVertex);
    }	
	printf("pre-cleanup src =%i out degree=%i\n", src, boost::out_degree(src, transitionSystem));
	ts_cleanup(); //remove self-edge and singleton states
    if (plan_tmp.empty() && (!transitionSystem[currentVertex].visited() || currentTask.is_over())){ //currentv not visited means that it wasn't observed ()
        printf("no plan, searchign from %i\n", src);
		printf("src =%i out degree=%i\n", src, boost::out_degree(src, transitionSystem));
        bool finished=false;
        ExecutionInfo info=package_info();
        plan_tmp= planner->plan(transitionSystem, src,info, &finished); //src
    }
    else{
        printf("recycled plan in explorer:\n");
    }
    m_plan=plan_tmp;
	//enforce_edge();
    printPlan(&m_plan);
}





std::pair<State, Edge> FocusedConfigurator::simulation_setup(b2World& w, Task & t, vertexDescriptor v0, b2Transform shift, b2Transform &start, std::vector<Direction>v0_options){
	start=b2Mul(shift, transitionSystem[v0].endPose);
	Disturbance Di=getDisturbance(transitionSystem, v0, w, v0_options[0], start);
	t = Task(Di, v0_options[0], start, true);//need to update end crit
	std::pair <State, Edge> sk(State(start, Di, v0_options[0]), Edge());
	adjust_simulated_task(v0, t);
	worldBuilder->buildWorld(w, t.getStart(), t.get_direction(), t.get_disturbance(), 0.15, WorldBuilder::PARTITION); //was g[v].endPose
	return sk;
}

void FocusedConfigurator::reassign_direction(vertexDescriptor bestNext, Direction& direction){
	std::vector <edgeDescriptor> best_in_edges= inEdges(bestNext);
	if (best_in_edges.empty()){
		direction=currentTask.get_direction();
	}
	else{
		direction = transitionSystem[bestNext].direction;
		transitionSystem[best_in_edges[0]].it_observed=iteration;
	}

}

// bool FocusedConfigurator::matchToSafe(VertexMatch &match,const  std::vector<VertexMatch>& other_matches){
// 	bool result=false;
// 	if (match.first==StateMatcher::_FALSE){
// 		return result;
// 	}
// 	if (transitionSystem[match.second].outcome!=simResult::crashed){
// 		return result;
// 	}
// 	for (VertexMatch m: other_matches){
// 		if (transitionSystem[m.second].outcome!=simResult::crashed){
// 			match.second=m.second;
// 			return true;
// 		}
// 	}
// 	return result;
// }

std::pair<edgeDescriptor, bool> FocusedConfigurator::setup_match_edge(VertexMatch &match, vertexDescriptor &v0, vertexDescriptor & v1,const Edge& k, Direction direction, bool changedMatch){
	v1=match.second; //frontier
	std::pair<edgeDescriptor, bool> edge= gt::add_edge(v0, v1, transitionSystem, iteration, direction); //assumes edge added
	if (edge.second){
		transitionSystem[edge.first]=k; //doesn't update motorstep
	}
	return edge;
}

std::vector <vertexDescriptor> FocusedConfigurator::task_vertices( vertexDescriptor v, std::pair<bool, edgeDescriptor>* ep){
	std::vector <vertexDescriptor> result= {v};
	Direction d=UNDEFINED;
	std::pair<bool, edgeDescriptor>ep2(false, edgeDescriptor()), _ep=ep2;
	do {
		std::vector <edgeDescriptor> ie=inEdges( v);
		ep2= gt::visitedEdge(ie, transitionSystem,v);
		if (!ep2.first){
			ep2=gt::getMostLikely(transitionSystem, ie, iteration);
		}
		if (ep2.first){
			if (ep2.second.m_target==result[0]){ //size 1
				_ep=ep2; //assign ep to define direction
				d= transitionSystem[_ep.second.m_target].direction;
				if (ep!=NULL){
					transitionSystem[_ep.second].it_observed=iteration;
				}
				for (edgeDescriptor e: ie){
					if (transitionSystem[e.m_target].direction==d && e!=ep2.second && 
						transitionSystem[e.m_source].Di == transitionSystem[_ep.second.m_source].Di &&
						transitionSystem[e.m_source].Dn == transitionSystem[_ep.second.m_target].Dn){
						ep2.second=e;
						break;
					}
				}
			}
			else if (transitionSystem[ep2.second.m_target].direction==d &&
			 	transitionSystem[ep2.second.m_target].Di == transitionSystem[_ep.second.m_target].Di &&
			 	transitionSystem[ep2.second.m_target].Dn == transitionSystem[_ep.second.m_target].Dn){ //same task!
				result.push_back(ep2.second.m_target); //source
			}
		}
		else{
			break;
		}
		v=ep2.second.m_source;
		if (ep2.second.m_target==currentVertex){ //source
			break;
		}
	}while(transitionSystem[ep2.second.m_target].direction==d);
	std::reverse(result.begin(), result.end());
	if (NULL!=ep){
		*ep=_ep;
	}
	return result;
}

std::vector <edgeDescriptor> FocusedConfigurator::inEdges(vertexDescriptor v, Direction d){
	std::vector <edgeDescriptor> result;
	auto es = boost::in_edges(v, transitionSystem);
	if (v==TransitionSystem::null_vertex()){
		return result;
	}
	if (transitionSystem[v].direction!=d && d!=UNDEFINED){
		return result;
	}
	for (auto ei = es.first; ei!=es.second; ++ei){
		//if (transitionSystem[(*ei).m_target].direction == d || d==UNDEFINED){
			if ((*ei).m_source!=v){
				result.push_back(*ei);
			}
		//}
	}
	return result;
}

vertexDescriptor FocusedConfigurator::getRecyclingStart(vertexDescriptor v, vertexDescriptor v1, vertexDescriptor taskStart){
	auto ies=inEdges(taskStart);
	vertexDescriptor result=v;
	if (transitionSystem[v1].outcome==simResult::crashed){
		int last_iteration=-1;
		for (auto ie: ies){
			if (ie.m_source!=v &&
			 transitionSystem[ie].it_observed>last_iteration){
				result=ie.m_source;
			}
		}
	}
	return result;
}

bool FocusedConfigurator::closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v){
	int MAX_OUT=3;
	if (transitionSystem[v].isTurning()){MAX_OUT=2;}
	std::vector<Direction> directions={UNDEFINED};
	if (getExploredDirections(v, directions).size()>=MAX_OUT){ 
		closed.emplace(v);
		return true;
	}
	return false;


}

std::pair<edgeDescriptor, bool> FocusedConfigurator::addEdgeRetrospectively(vertexDescriptor v, vertexDescriptor & v1, const State & s_tmp, std::pair<edgeDescriptor, bool> first_edge, Direction d, float linearSpeed){
	transitionSystem[v].options = {d};
	transitionSystem[v].endPose=s_tmp.endPose;
	transitionSystem[v].Dn=s_tmp.Dn;
	transitionSystem[first_edge.first].step= gt::distanceToSimStep(transitionSystem[v].distance(), linearSpeed);
	first_edge=add_vertex_retro(v, v1); 
	transitionSystem[v1].Di=transitionSystem[v].Di;
	transitionSystem[v1].start=transitionSystem[v].endPose;
	transitionSystem[v].phi=NAIVE_PHI;
	transitionSystem[v1].direction=d;
	transitionSystem[v].outcome=simResult::safeForNow;
	return first_edge;
}

void FocusedConfigurator::correctQueue(std::vector<vertexDescriptor>& queue, vertexDescriptor v, vertexDescriptor startRecycle, int planProvSize){
	if (planProvSize==0 || startRecycle==v){
		return;
	}
	auto v_it=check_vector_for(queue, v);
	*v_it=startRecycle;

}

void FocusedConfigurator::adjustProbability(const edgeDescriptor &e){
	if (e.m_target==TransitionSystem::null_vertex()){
		return;
	}
	std::vector <edgeDescriptor> es=gt::outEdges(transitionSystem, e.m_source, transitionSystem[e.m_target].direction);
	float totObs=0;
	//find total observations
	for (edgeDescriptor & ei:es){
		transitionSystem[ei].probability=transitionSystem[ei.m_target].nObs/es.size();
		 	totObs+=transitionSystem[ei.m_target].nObs;
	}
	//adjust
	for (edgeDescriptor &ei: es){
		transitionSystem[ei].probability=transitionSystem[ei.m_target].nObs/totObs;
	}
}

void FocusedConfigurator::abandonPlan(std::vector<vertexDescriptor>& planProv, vertexDescriptor v0, vertexDescriptor v1){
	if (current_vertices[0]!=DUMMY){ //dummy
		current_vertices.clear();
	}
	planProv.clear();
	currentTask.set_change(true);


}

std::vector<Direction> FocusedConfigurator::partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve){
	std::vector <Direction> result;
	if (ve.first){
		if(transitionSystem[ve.second.m_target].visited()){
			//only simulate current task
			if (transitionSystem[ve.second.m_target].outcome!=simResult::crashed){
				return {currentTask.get_direction()};
			}
			//preferred task fails: plan evasion maneuvre
			else if (transitionSystem[ve.second.m_target].outcome==simResult::crashed){
			result={DEFAULT, LEFT, RIGHT};
			erase_from_vector(result, currentTask.get_direction());
			return result;
			}
	}
}
return result;
}

Robot FocusedConfigurator::makeRobot(b2World & world, const b2Transform & start){
	Robot robot=Configurator::makeRobot(world, start);
	b2AABB sensor_aabb=worldBuilder->makeRobotSensor(robot.body(), controlGoal.get_disturbance()); //Configurator::getGoalDisturbance()
	return robot;
}

void FocusedConfigurator::EvaluationQueueManager::addToEvaluationQueue(std::vector <vertexDescriptor>& evaluationQueue, vertexDescriptor v1, TransitionSystem & g, vertexDescriptor v){
	if (g[v].direction==STOP || g[v1].direction==DEFAULT){
		evaluationQueue.push_back(v1);
	}
	if (g[v1].outcome==simResult::successful){
		if (lastAdded!=TransitionSystem::null_vertex()){
			if (boost::edge(lastAdded, v1, g).second ){ //if edge exists
				erase_from_vector(evaluationQueue, lastAdded);
			}
		}
	}
	lastAdded=v1;
}

void FocusedConfigurator::enforce_edge(){
	if (m_plan.empty()){return;}
	auto edge=gt::add_edge(MOVING_VERTEX, m_plan[0], transitionSystem, iteration);
}

bool FocusedConfigurator::shouldPartiallyExplore(const std::vector<edgeDescriptor>& oe, std::pair<bool, edgeDescriptor> ve){
	return ( !currentTask.get_change() ||!oe.empty()) && iteration>1;
}