#include "configurator.h"
#include <chrono>

void Configurator::MulT(const b2Transform& B, Task& task){
	math::MulT(B, task.start);
	math::MulT(B, task.disturbance);
}

void Configurator::Mul(const b2Transform&B , Task &task){
	task.start=b2Mul(B, task.start);
	task.disturbance.bf.pose=b2Mul(B, task.disturbance.pose());

}


void Configurator::init(Task _task){
	controlGoal=_task;
	currentTask=_task;
	register_tracker(tracker);
	//previousTimeScan = std::chrono::high_resolution_clock::now();
	//MOVING_VERTEX=boost::add_vertex(transitionSystem);
	transitionSystem[MOVING_VERTEX].Di=controlGoal.disturbance;
	currentVertex=MOVING_VERTEX;
	//boost::add_edge(MOVING_VERTEX, currentVertex,transitionSystem);
	currentTask.action.setVelocities(0,0);
	currentTask.set_change(1);
	gt::fill(simResult(), &transitionSystem[MOVING_VERTEX]);

}

void Configurator::dummy_vertex(vertexDescriptor src){
	vertexDescriptor prev_current=currentVertex;
	currentVertex=boost::add_vertex(transitionSystem);
	gt::fill(simResult(), &transitionSystem[currentVertex]);
	transitionSystem[currentVertex].nObs++;
	transitionSystem[currentVertex].Di=controlGoal.disturbance;
	currentTask=Task(controlGoal.disturbance, Direction::STOP, b2Transform_zero, true);
	movingEdge = boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem).first;
	currentEdge = boost::add_edge(src, currentVertex, transitionSystem).first;
	transitionSystem[movingEdge].it_observed=iteration;
	transitionSystem[currentEdge].it_observed=iteration;
	// printf("dummy, current edge = %i, %i\n", src, currentVertex);
	transitionSystem[MOVING_VERTEX].direction=STOP;
	transitionSystem[currentVertex].direction=STOP;
}

std::pair <edgeDescriptor, bool> AttentiveConfigurator::add_vertex_now(const vertexDescriptor & src, vertexDescriptor &v1, Disturbance Di,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, edge, topDown);
	if (!transitionSystem[v1].filled){
		transitionSystem[v1].Di= Di;
	}
	return result;
}

std::pair <edgeDescriptor, bool> AttentiveConfigurator::add_vertex_retro(vertexDescriptor & src, vertexDescriptor &v1,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, edge, topDown);
	transitionSystem[v1].Di= transitionSystem[src].Di;
	transitionSystem[v1].Dn=transitionSystem[src].Dn;
	return result;
}




bool Configurator::Spawner(){ 
	iteration++; //iteration set in getVelocity
	worldBuilder.add_iteration();

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();

	//CREATE BOX2D ENVIRONMENT
	b2World world= b2World(GRAVITY);
	char name[256];
	worldBuilder.set_world_objects(worldBuilder.getFeatures(data2fp, b2Transform_zero, WorldBuilder::PARTITION));
	// printf("got features =%i\n", worldBuilder.world_objects.size());	
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d_getFeatures= now- endTime; //in seconds
	float duration_getFeatures=abs(float(d_getFeatures.count())/1000); //express in seconds
	// printf("built wolrd in %f\n", duration);
	explore_plan(world);
	endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d_withExplore= now- endTime; //in seconds
	float duration_withExplore=abs(float(d_withExplore.count())/1000); //express in seconds
	//FORMAT: vertices	bodies	total_dur	just_worldbuilding
	if (logger){
		logger->log("%i\t%i\t%0.6f\t%0.6f\n", transitionSystem.m_vertices.size(), worldBuilder.bodies, duration_withExplore, duration_getFeatures);
	}
	worldBuilder.resetBodies();
	return 1;
}

void AttentiveConfigurator::resetPhi(){
	auto vs=boost::vertices(transitionSystem);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		transitionSystem[*vi].resetVisited();
		transitionSystem[*vi].options.clear();
	}
}



std::pair <bool, Direction> AttentiveConfigurator::getOppositeDirection(Direction d){
	std::pair <bool, Direction> result(false, DEFAULT);
		switch (d){
		case Direction::LEFT: result.first = true; result.second = RIGHT;break;
		case Direction::RIGHT: result.first = true; result.second = LEFT;break;
		default:
		break;
	}
	return result;
}
Disturbance AttentiveConfigurator::getDisturbance(TransitionSystem&g,vertexDescriptor v, b2World & world, const Direction& dir, const b2Transform& start){
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=inEdges(v);
		std::vector <edgeDescriptor> out=gt::outEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);
			if (visited.first ||out.empty()){
				if (g[v].Di.isValid() && g[v].Di.getAffIndex()==AVOID && g[v].direction!=dir){
					Task task(g[v].Di, DEFAULT, g[v].endPose, true);
					Robot robot(&world);
					robot.body->SetTransform(task.getStart().p, task.getStart().q.GetAngle());
					b2AABB box =worldBuilder.makeRobotSensor(robot.body, controlGoal.get_disturbance_ptr());
					b2Fixture *sensor =GetSensor(robot.body);
					bool overlap=overlaps(robot.body, &g[v].Di) && sensor;
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
	//return controlGoal.disturbance;
}





simResult Configurator::simulate(Task  t, b2World & w){ //State& state, State src, 
		//EVALUATE NODE()
	simResult result;
	float distance=BOX2DRANGE;
	if (controlGoal.disturbance.isValid()){
		distance= controlGoal.disturbance.getPosition().Length();
	}
	float remaining=distance/controlGoal.action.getLinearSpeed();
	Robot robot(&w);
	worldBuilder.add_body_count();
	robot.body->SetTransform(t.start.p, t.start.q.GetAngle());
	b2AABB sensor_aabb=worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
	result =t.bumping_that(w, iteration, robot.body, remaining); //default start from 0
	//approximate angle to avoid rounding errors
	float approximated_angle=approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode);
	result.endPose.q.Set(approximated_angle);
	return result;
	}


std::vector<vertexDescriptor> AttentiveConfigurator::explorer(vertexDescriptor v, TransitionSystem& g, b2World & w){
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
				er  = Planner::estimateCost(sk.first, g[v0].endPose, sk.first.direction,controlGoal);
				StateDifference sd;
				std::vector <VertexMatch> other_matches;
				VertexMatch match=findMatch(sk.first, t.get_direction(), StateMatcher::MATCH_TYPE::ABSTRACT, &sd, &other_matches);		//, closest_match	
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher.match_equal(match.first,StateMatcher::MATCH_TYPE::ABSTRACT)){
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
				g[v1].phi=Planner::evaluationFunction(er, v1, plan_prov);
				propagateD(v1, v0, &closed); //if v0 is a dummy vertex it propagates the disturbance
				v0_exp=v0;					
				options=g[v0_exp].options;
				v0=v1;
				evaluationQueue.push_back(v1);						
			}while(t.get_direction() !=DEFAULT & int(g[v0].options.size())!=0);
		evaluationQueue.push_back(v1);
		}
	}
	backtrack(evaluationQueue, priorityQueue, closed, plan_prov, v, startRecycle);
	bestNext=priorityQueue[0];
	reassign_direction(bestNext, direction);
}while(g[bestNext].options.size()>0 && !er.ended);
// printf("finished exploring, plan =%i\n", plan_prov.size());
return plan_prov;
}

std::vector <vertexDescriptor> AttentiveConfigurator::splitTask( vertexDescriptor v,  Direction d, vertexDescriptor src){
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
	auto ie=inEdges(src);
	auto sameIterationEdgeIt=check_vector_for(ie, SameIteration(transitionSystem, iteration));
	if (!transitionSystem[src].isTurning()&& !ie.empty()){ //! //&& sameIterationEdgeIt!=ie.end()
		transitionSystem[src].outcome=simResult::safeForNow;
		split.insert(split.begin(), src);
	}
	vertexDescriptor v1=v;
	float nNodes = transitionSystem[v].distance()/simulationStep, og_phi=transitionSystem[v].phi;
	b2Transform endPose = transitionSystem[v].endPose;
	Task::Action a;
	a.init(d);
	b2Transform deltaTransform=b2Transform_zero;
	deltaTransform.p.x=simulationStep;
	while(nNodes>1){
		State s_tmp=State(transitionSystem[v]);
		if(nNodes >1){
			s_tmp.endPose=b2Mul(transitionSystem[v].start, deltaTransform);
			VertexMatch match=findMatch(s_tmp, d);
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
			VertexMatch match=findMatch(s_tmp, d);
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


void AttentiveConfigurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src, vertexDescriptor startRecycle){
	for (vertexDescriptor v:evaluation_q){
		std::pair<bool, edgeDescriptor> ep(false, edgeDescriptor());
		std::vector <vertexDescriptor> split = task_vertices(v, &ep); 
		Direction direction= transitionSystem[ep.second.m_target].direction;
		if (split.size()<2){
			split =splitTask(v, DEFAULT, ep.second.m_source);
		}
		correctQueue(split, module_src, startRecycle, plan_prov.size());
		// for (int i=0; i<split.size(); i++){ //
		for (int i=split.size()-1; i>=0; i--){ //
			vertexDescriptor split_v=split[i], src=TransitionSystem::null_vertex();
			if (i<1){
				auto ep=gt::getMostLikely(transitionSystem, inEdges(split_v), iteration);
				if (ep.first){
					src=ep.second.m_source;
				}
				else{
					src=split_v;
				}
			}
			else{
				src=split[i-1];
			}
			EndedResult local_er=Planner::estimateCost(transitionSystem[split_v],transitionSystem[split_v].start,direction, controlGoal);
			transitionSystem[split_v].phi=Planner::evaluationFunction(local_er, split_v, plan_prov);
			applyTransitionMatrix(split_v, direction, local_er.ended,src, plan_prov);
			addToPriorityQueue(split_v, priority_q, closed);
			src=split_v;
		}
	}
	evaluation_q.clear();

}

void AttentiveConfigurator::propagateD(vertexDescriptor v1, vertexDescriptor v0, std::set <vertexDescriptor>*closed,StateMatcher::MATCH_TYPE match){
	if (transitionSystem[v1].outcome == simResult::successful || !boost::edge(v0, v1, transitionSystem).second){
		return;
	}
	bool same_Di=transitionSystem[v0].Di==transitionSystem[v1].Di;
	bool shouldBeUpdated= transitionSystem[v0].direction==STOP;
	if (shouldBeUpdated&& same_Di && transitionSystem[v0].Dn.getAffIndex()==NONE){
 			transitionSystem[v0].Dn = transitionSystem[v1].Dn; //was target
 	}
	if (v1==currentVertex){
		transitionSystem[v0].outcome=simResult::safeForNow;
	}
	return;
}

std::pair<edgeDescriptor, bool> Configurator::addVertex(const vertexDescriptor & src, vertexDescriptor &v1, Edge edge, bool topDown){ //returns edge added
	std::pair<edgeDescriptor, bool> result;
	result.second=false;
	if (transitionSystem[src].options.size()>0 || topDown){
		v1 = boost::add_vertex(transitionSystem);
		result = add_edge(src, v1, transitionSystem);
		transitionSystem[result.first] =edge;
		transitionSystem[v1].direction=transitionSystem[src].options[0];
		transitionSystem[result.first].it_observed=iteration;
		if (!topDown){
			transitionSystem[src].options.erase(transitionSystem[src].options.begin());
		}

	}
	return result;
}




void Configurator::printPlan(std::vector <vertexDescriptor>* p){

	std::vector <vertexDescriptor> _plan= m_plan;
	if (p){
	_plan=*p;		
	}
	vertexDescriptor pre=currentVertex;
	printf("PLAN:");
	for (vertexDescriptor v: _plan){
		auto a=dirmap.find(transitionSystem[v].direction);
		printf("%i, %s; ", v, (*a).second); //, transitionSystem[edge.first].step	
		}
	printf("\n");
}



void Configurator::start(){
	if (ci == NULL){
		throw std::invalid_argument("no LIDAR interface found");
		return;
	}
	if (control==NULL){
		throw std::invalid_argument("no motor interface found");
		return;
	}
	running =1;
	if (LIDAR_thread!=NULL){ //already running
		return;
	}
	LIDAR_thread= new std::thread(Configurator::run, this);
}

void Configurator::stop(){
	running =0;
	if (LIDAR_thread!=NULL){
		LIDAR_thread->join();
		delete LIDAR_thread;
		LIDAR_thread=NULL;
	}

}

void Configurator::registerInterface(LIDAR_In * _ci, Motor_Out * _control){
	ci = _ci;
	control=_control;
}

void Configurator::run(Configurator * c){
	while (c->running){
		if (c->ci->stop){
			c->ci=NULL;
			c->control=NULL;
			printf("ci not started\n");
		}
		if (c->ci == NULL){
			printf("null pointer to lidar input\n");
			c->running=0;
			return;
		}
		if (c->control == NULL){
			printf("null pointer to motor output\n");
			c->running=0;
			return;
		}
		if (c == NULL){
			printf("null pointer to configurator\n");
			c->running=0;
			return;
		}
		if (c->task_controller==NULL){
			c->running=0;
			throw std::invalid_argument("no task controller, please set!");
		}
		if (c->ci->isReady()){
			c->ci->setReady(false);
			c->data2fp= CoordinateContainer(c->ci->data2fp);
			c->Spawner();
			b2Transform deltaPose=b2Transform_zero;
			if (c->getIteration()>1){
				deltaPose= c->tracker->track((c->currentTask),c->ci->data2fp, c->worldBuilder.get_world_objects());
			}
			c->update_graph(c->transitionSystem, deltaPose);
			if (c->goal_changer!=NULL){
				if (( c->currentTask.is_over()& c->transitionSystem[c->currentVertex].direction!=STOP && c->m_plan.empty() && c->getIteration()>1)){
					c->goal_changer->change_goal(&c->controlGoal);
				}					
			}
			c->change_task();		
			c->adjust_goal_expectation();
			c->estimate_current_vertex();
			printf("current v=%i\n", c->currentVertex);
			c->tracker->on_new_reading(&c->controlGoal);
			}

	}

}

std::vector <Direction>  AttentiveConfigurator::getExploredDirections(vertexDescriptor v, const std::vector<Direction>& directions){
	std::vector <Direction> result;
	for (Direction direction: directions){
		for (edgeDescriptor &e: gt::outEdges(transitionSystem, v, direction)){
			if (transitionSystem[e].it_observed==iteration){ //g[e.m_target].visited()
				result.push_back(transitionSystem[e.m_target].direction);
			}
		}
	}
	return result;
}



void AttentiveConfigurator::removeExploredTransitions( vertexDescriptor v){
	std::vector<Direction>options=getExploredDirections(v, transitionSystem[v].options);
	for (Direction d:options){
		erase_from_vector(transitionSystem[v].options, d);
	}
}

void AttentiveConfigurator::transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src){
	Task temp(controlGoal.get_disturbance(), DEFAULT, transitionSystem[v].endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	auto oe=gt::outEdges(transitionSystem, v, d);
	if (( !currentTask.get_change() ||!oe.empty()) && (iteration>1)){
		std::pair<bool, edgeDescriptor> ve=gt::visitedEdge(oe, transitionSystem, currentVertex);
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

void AttentiveConfigurator::applyTransitionMatrix(vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov){
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


void AttentiveConfigurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, const std::set <vertexDescriptor>& closed){
	if (transitionSystem[v].outcome==simResult::crashed){
		return;
	}
	auto found=closed.find(v); 
	if(found==closed.end()){ //if not in closed
		for (auto i =queue.begin(); i!=queue.end(); i++){
			bool expanded=0;
			if (transitionSystem[v].phi <abs(transitionSystem[*i].phi) ){
				queue.insert(i, v);
				return;
			}
		}
		queue.push_back(v);		
	}
}


std::pair <edgeDescriptor, bool> AttentiveConfigurator::maxProbability(std::vector<edgeDescriptor> ev, TransitionSystem& g){
	std::pair <edgeDescriptor, bool> result;
	if (ev.empty()){
		result.second=false;
		return result;
	}
	result.first = ev[0];
	result.second=true;
	for (edgeDescriptor e :ev){
		if (g[e].probability>g[result.first].probability){
			result.first =e;
		}
	}
	return result;
}

void AttentiveConfigurator::adjust_simulated_task(const vertexDescriptor &v, Task & t){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, transitionSystem);
	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	if (!t.getEndCriteria().angle.isValid()){return;}
	if (t.get_direction()==DEFAULT){return;}
	if (t.get_direction()==currentTask.get_direction()){
		t.getEndCriteria().adjust(-tracker->getDeltaTransform());
	}
	else if (t.get_direction()==getOppositeDirection(currentTask.get_direction()).second){
		t.getEndCriteria().adjust(tracker->getDeltaTransform());
	}
}



// void AttentiveConfigurator::adjust_rw_task(const vertexDescriptor &v, TransitionSystem &g, Task * t, const b2Transform & deltaPose){
// 	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

// 	if(!ep.second){ //no tgt	
// 		if (v==0){
// 			printf("edge doesn't exist");
// 		}
// 		return; //check until needs to be checked
// 	}
// 	// auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
// 	// int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
// 	// float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
// 	// float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
// 	if (t->getAction().getOmega()!=0){
// 		float remainingAngle = t->getEndCriteria().angle.get()-abs(deltaPose.q.GetAngle());
// 	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
// 		// if (t->direction==getOppositeDirection(t->direction).second){
// 		// 	remainingAngle=M_PI-remainingAngle;
// 		// }
// 		t->setEndCriteria(Angle(remainingAngle));
// 	}
// 	if(t->getAction().getLinearSpeed()>0){
// 		//step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
// 		t->setEndCriteria(Distance(t->getEndCriteria().distance.get()-deltaPose.p.Length()));
// 	}			// -estimated distance covered

// }




VertexMatch AttentiveConfigurator::findMatch(State s, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd, std::vector <VertexMatch>*other_matches){
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
			if (auto vertices=task_vertices(v); vertices.size()>1){
				q.start=transitionSystem[vertices[0]].start;
			}			
		if (v==currentVertex && !currentTask.is_over()){
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
			if (condition){
				result.first= m;
				result.second=v;
				VertexMatchTuple to_add(v, m, sum_tmp);
				others_set.emplace(to_add);
			}
			if (sum_tmp<sum){
				sum=sum_tmp;
				result.first=m;
				result.second=v;			
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
	if (NULL!=other_matches){
		for (VertexMatchTuple item:others_set){
			other_matches->push_back(VertexMatch(std::get<1>(item), std::get<0>(item)));
		}
	}
	return result;
}



void AttentiveConfigurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:m_plan){
		if (p==v){
       		g[v].phi-=.1;
			break;
		}
    } 
}



float Configurator::approximate_angle(const float & angle, const Direction & d, const simResult::resultType & outcome){
	float result=angle, decimal, integer;
	if ((d==LEFT || d==RIGHT)&& outcome!=simResult::crashed){
		float ratio= angle/ANGLE_RESOLUTION;
		decimal=std::modf(ratio, &integer);
		if (fabs(decimal)>=0.5){
			if (integer<0){
				integer-=1;
			}
			else{
				integer+=1;
			}
		}		
		result=integer*ANGLE_RESOLUTION;
	}
	return result;
}


void AttentiveConfigurator::ts_cleanup(TransitionSystem & g, std::vector <vertexDescriptor>& p){
	FilteredTS fts(transitionSystem, ViableEdge(&transitionSystem), Connected(&transitionSystem)); //boost::keep_all()
	TransitionSystem tmp;
	boost::copy_graph(fts, tmp);	
	transitionSystem.clear();	
	transitionSystem.swap(tmp);		
}
 
void AttentiveConfigurator::shift_states(TransitionSystem & g, const std::vector<vertexDescriptor>& p, const b2Transform & shift_start){
	if (p.empty()){
		return;
	}
	for (const vertexDescriptor &v:p){
		math::MulT(shift_start, g[v]);
	}
}

vertexDescriptor AttentiveConfigurator::get_explore_start(TransitionSystem & g){
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

void AttentiveConfigurator::pre_explore(){
	//if (MOVING_VERTEX!=currentVertex){
		boost::remove_out_edge_if(MOVING_VERTEX, is_not_v(currentVertex), transitionSystem);
	//}	
		//transitionSystem[MOVING_VERTEX].Di=currentTask.get_disturbance();
		transitionSystem[MOVING_VERTEX].Di=transitionSystem[currentVertex].Di;
		transitionSystem[MOVING_VERTEX].outcome=simResult::successful;
		movingEdge=boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem).first;
	//  if (currentTask.get_change()){
	//  	transitionSystem[movingEdge].step=currentTask.getMotorStep();
	//  }
}


void Configurator::estimate_current_vertex(){
	if(current_vertices.empty()){
		currentVertex=MOVING_VERTEX;
		return;
	}
	if (current_vertices.size()==1){
		currentVertex=current_vertices[0];
		auto e=boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem);
		movingEdge=e.first;
		return;
	}
	vertexDescriptor task_start=current_vertices[0], cv=TransitionSystem::null_vertex();
	b2Transform Di_distance=currentTask.from_Di(), v_from_D=b2Transform_zero;
	float sum=10000;
	StateMatcher matcher;
	for (vertexDescriptor & v:current_vertices){
		if ((transitionSystem[task_start].Dn.getAffIndex()==AVOID && currentTask.disturbance.getAffIndex()==PURSUE)){
			v_from_D=transitionSystem[v].start_from_Dn();
		}
		else{
			v_from_D=transitionSystem[v].start_from_Di();
		}
		b2Transform transform_diff=Di_distance-v_from_D;
		float sum_diff=fabs(transform_diff.p.x+transform_diff.p.y+transform_diff.q.GetAngle());
		if (sum_diff<sum){
			cv=v;
			sum=sum_diff;
		}				
	}
	printf("current vertex cv=%i\n", cv);
	currentVertex=cv;

}


void Configurator::change_task(){
	if (!currentTask.is_over()){
		return;
	}
	if (task_controller==NULL){
		throw std::invalid_argument("no controller, please add!");
	}
	task_controller->next_task(currentTask, controlGoal, transitionSystem, current_vertices, m_plan);
	//transitionSystem[movingEdge].step=currentTask.getMotorStep();
	std::cout<<"new task step= "<<currentTask.getMotorStep()<<std::endl;
	tracker->on_new_task(&currentTask);
	control->reset();
	control->getData(currentTask.action);
	return;
}

void Configurator::update_graph(TransitionSystem&g, const b2Transform & _deltaPose){
	math::MulT(_deltaPose, g);
	Configurator::MulT(_deltaPose, controlGoal);
}


void Configurator::adjust_goal_expectation(){
	if (controlGoal.getAffIndex()==PURSUE && !m_plan.empty()&&task_controller->get_disturbance().getAffIndex()!=NONE){
		b2Transform from_Di=b2Transform_zero;
		//if (task_controller->get_disturbance().getAffIndex()==AVOID){
		from_Di=currentTask.from_Di();
		//}
		//b2Transform sum_transform=from_Di+task_controller->to_goal(); //where goal should be
		b2Transform goal_robotPOV= b2Mul(from_Di,task_controller->disturbance_to_goal()); //position of goal from the robot based on where it should be from Di
		controlGoal.disturbance.bf.pose=goal_robotPOV;
		// debug::print_pose(b2MulT(from_Di, sum_transform), "from Di to sum transform:");
		// debug::print_pose(b2Mul(from_Di,task_controller->to_goal()), "from Di mulT to goal:");
// 		b2Transform difference=controlGoal.disturbance.pose()-sum_transform; //difference in pose
// //		debug::print_pose(difference, "difference between pose and likely goal pose:");
// 		math::MulT(difference, &controlGoal);//update goal with ratio info
		debug::print_pose(controlGoal.disturbance.pose(), "goal after adjusting");
		printf("distance after adjusting %f\n", controlGoal.disturbance.pose().p.Length());
	}

}

void AttentiveConfigurator::explore_plan(b2World&world){
    pre_explore();
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi();
	transitionSystem[MOVING_VERTEX].phi=Planner::evaluationFunction(EndedResult(), MOVING_VERTEX, m_plan);
    std::vector <vertexDescriptor> plan_tmp=explorer(src, transitionSystem, world);
    if (DEBUG){
        std::vector<vertexDescriptor> _plan=(m_plan);
        debug::graph_file(iteration, transitionSystem, controlGoal.get_disturbance(), _plan, currentVertex);
    }	
	try{
		ts_cleanup(transitionSystem, m_plan); //remove self-edge and singleton states
	}
	catch(...){}	
    if (plan_tmp.empty() && (!transitionSystem[currentVertex].visited() || currentTask.is_over())){ //currentv not visited means that it wasn't observed ()
        printf("no plan, searchign from %i\n", src);
        bool finished=false;
        ExecutionInfo info=package_info();
        plan_tmp= planner->plan(transitionSystem, currentVertex,info, &finished); //src
    }
    else{
        printf("recycled plan in explorer:\n");
    }
    m_plan=plan_tmp;
    printPlan(&m_plan);
}

void ReactiveConfigurator::explore_plan(b2World &world){
	if (transitionSystem.m_vertices.size()==1 && iteration<=1){
		movingEdge = boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem).first;
		transitionSystem[MOVING_VERTEX].direction=DEFAULT;
		currentTask.getAction().init(transitionSystem[currentVertex].direction);
	}
	if (currentTask.getAction().getOmega()!=0 && currentTask.getMotorStep()<(transitionSystem[movingEdge].step)){
		return;
	}
	//adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
	worldBuilder.buildWorld(world, transitionSystem[MOVING_VERTEX].start, currentTask.get_direction()); //was g[v].endPose
	Task t=currentTask;
	t.H(t.get_disturbance(), t.get_direction(), true);
	simResult result = simulate(t, world); //transitionSystem[currentVertex],transitionSystem[currentVertex],
	gt::fill(result, &transitionSystem[currentVertex], &transitionSystem[currentEdge]);
	
	//transitionSystem[currentVertex].Dn.set_affordance(as.affordance);
	currentTask.set_change(transitionSystem[currentVertex].outcome!=simResult::successful);
	if (currentTask.get_change()){
		printf("crashed\n");
	}
}


bool AttentiveConfigurator::recycle_plan(vertexDescriptor v, vertexDescriptor &v0, vertexDescriptor & task_start, StateMatcher::MATCH_TYPE& matchType, 
											b2Transform & shift_start, b2Transform& sk_first_start, std::pair<edgeDescriptor, bool>&edge, 
											std::vector<vertexDescriptor> &plan_prov, Direction t_get_direction){
	bool finished=false, result=false;
	bool been = matchType==StateMatcher::ABSTRACT || matchType==StateMatcher::_TRUE;
	Task controlGoal_adjusted= controlGoal;
	//position of task start with respect to goal disturbance (pov)
	shift_start= b2MulT(b2MulT(sk_first_start, controlGoal.getStart()), transitionSystem[task_start].start);
	Mul(shift_start, controlGoal_adjusted);
	boost::remove_edge(edge.first, transitionSystem);
	edge= gt::add_edge(v0, task_start, transitionSystem, iteration, transitionSystem[edge.first.m_target].direction);
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

std::pair<State, Edge> AttentiveConfigurator::simulation_setup(b2World& w, Task & t, vertexDescriptor v0, b2Transform shift, b2Transform &start, std::vector<Direction>v0_options){
	start=transitionSystem[v0].endPose +shift;
	Disturbance Di=getDisturbance(transitionSystem, v0, w, v0_options[0], start);
	t = Task(Di, v0_options[0], start, true);//need to update end crit
	std::pair <State, Edge> sk(State(start, Di, v0_options[0]), Edge());
	adjust_simulated_task(v0, t);
	worldBuilder.buildWorld(w, t.getStart(), t.get_direction(), t.get_disturbance(), 0.15, WorldBuilder::PARTITION); //was g[v].endPose
	return sk;
}

void AttentiveConfigurator::reassign_direction(vertexDescriptor bestNext, Direction& direction){
	std::vector <edgeDescriptor> best_in_edges= inEdges(bestNext);
	if (best_in_edges.empty()){
		direction=currentTask.get_direction();
	}
	else{
		direction = transitionSystem[bestNext].direction;
		transitionSystem[best_in_edges[0]].it_observed=iteration;
	}

}

bool AttentiveConfigurator::matchToSafe(VertexMatch &match,const  std::vector<VertexMatch>& other_matches){
	bool result=false;
	if (match.first==StateMatcher::_FALSE){
		return result;
	}
	if (transitionSystem[match.second].outcome!=simResult::crashed){
		return result;
	}
	for (VertexMatch m: other_matches){
		if (transitionSystem[m.second].outcome!=simResult::crashed){
			match.second=m.second;
			return true;
		}
	}
	return result;
}

std::pair<edgeDescriptor, bool> AttentiveConfigurator::setup_match_edge(VertexMatch &match, vertexDescriptor &v0, vertexDescriptor & v1,const Edge& k, Direction direction, bool changedMatch){
	v1=match.second; //frontier
	std::pair<edgeDescriptor, bool> edge= gt::add_edge(v0, v1, transitionSystem, iteration, direction); //assumes edge added
	if (edge.second){
		transitionSystem[edge.first]=k; //doesn't update motorstep
	}
	return edge;
}

std::vector <vertexDescriptor> AttentiveConfigurator::task_vertices( vertexDescriptor v, std::pair<bool, edgeDescriptor>* ep){
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

std::vector <edgeDescriptor> AttentiveConfigurator::inEdges(vertexDescriptor v, Direction d){
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

vertexDescriptor AttentiveConfigurator::getRecyclingStart(vertexDescriptor v, vertexDescriptor v1, vertexDescriptor taskStart){
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

bool AttentiveConfigurator::closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v){
	int MAX_OUT=3;
	if (transitionSystem[v].isTurning()){MAX_OUT=2;}
	std::vector<Direction> directions={UNDEFINED};
	if (getExploredDirections(v, directions).size()>=MAX_OUT){
		closed.emplace(v);
		return true;
	}
	return false;


}

std::pair<edgeDescriptor, bool> AttentiveConfigurator::addEdgeRetrospectively(vertexDescriptor v, vertexDescriptor & v1, const State & s_tmp, std::pair<edgeDescriptor, bool> first_edge, Direction d, float linearSpeed){
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

void AttentiveConfigurator::correctQueue(std::vector<vertexDescriptor>& queue, vertexDescriptor v, vertexDescriptor startRecycle, int planProvSize){
	if (planProvSize==0 || startRecycle==v){
		return;
	}
	auto v_it=check_vector_for(queue, v);
	*v_it=startRecycle;

}

void AttentiveConfigurator::adjustProbability(const edgeDescriptor &e){
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

void AttentiveConfigurator::abandonPlan(std::vector<vertexDescriptor>& planProv, vertexDescriptor v0, vertexDescriptor v1){
	planProv.clear();
	if (v0==MOVING_VERTEX && transitionSystem[v1].direction==currentTask.get_direction()){
		currentTask.set_change(true);
		current_vertices.clear();
		//currentVertex=MOVING_VERTEX;
	}
}

std::vector<Direction> AttentiveConfigurator::partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve){
	std::vector <Direction> result;
	if (ve.first){
	if(transitionSystem[ve.second.m_target].visited()){
		if (transitionSystem[ve.second.m_target].outcome!=simResult::crashed){
			return {currentTask.get_direction()};
		}
		else if (transitionSystem[ve.second.m_target].outcome==simResult::crashed){
		result={DEFAULT, LEFT, RIGHT};
		erase_from_vector(result, currentTask.get_direction());
		return result;
		}
	}
}
return result;
}

void AttentiveConfigurator::EvaluationQueueManager::addToEvaluationQueue(std::vector <vertexDescriptor>& evaluationQueue, vertexDescriptor v, TransitionSystem & g){
	evaluationQueue.push_back(v);
	if (g[v].outcome==simResult::successful){
		if (lastAdded==TransitionSystem::null_vertex()){
			return;
		}
		if (boost::edge(lastAdded, v, g).second){ //if edge exists
			erase_from_vector(evaluationQueue, lastAdded);
		}
	}
	lastAdded=v;


}
