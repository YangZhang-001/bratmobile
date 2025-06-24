#include "configurator.h"
#include <chrono>

void Configurator::init(Task _task){
	controlGoal=_task;
	currentTask=_task;
	register_tracker(tracker);
	//previousTimeScan = std::chrono::high_resolution_clock::now();
	//movingVertex=boost::add_vertex(transitionSystem);
	transitionSystem[movingVertex].Di=controlGoal.disturbance;
	currentVertex=movingVertex;
	//boost::add_edge(movingVertex, currentVertex,transitionSystem);
	currentTask.action.setVelocities(0,0);
	currentTask.set_change(1);
	gt::fill(simResult(), &transitionSystem[movingVertex]);

}

void Configurator::dummy_vertex(vertexDescriptor src){
	vertexDescriptor prev_current=currentVertex;
	currentVertex=boost::add_vertex(transitionSystem);
	gt::fill(simResult(), &transitionSystem[currentVertex]);
	transitionSystem[currentVertex].nObs++;
	transitionSystem[currentVertex].Di=controlGoal.disturbance;
	currentTask=Task(controlGoal.disturbance, Direction::STOP, b2Transform_zero, true);
	movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
	currentEdge = boost::add_edge(src, currentVertex, transitionSystem).first;
	// printf("dummy, current edge = %i, %i\n", src, currentVertex);
	transitionSystem[movingVertex].direction=STOP;
	transitionSystem[currentVertex].direction=STOP;
}

std::pair <edgeDescriptor, bool> AttentiveConfigurator::add_vertex_now(const vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Disturbance Di,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, g, edge, topDown);
	if (!g[v1].filled){
		g[v1].Di= Di;
	}
	return result;
}

std::pair <edgeDescriptor, bool> AttentiveConfigurator::add_vertex_retro(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, g, edge, topDown);
	g[v1].Di= g[src].Di;
	g[v1].Dn=g[src].Dn;
	return result;
}




bool Configurator::Spawner(){ 
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++; //iteration set in getVelocity
	worldBuilder.add_iteration();

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();

	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];
	worldBuilder.set_world_objects(worldBuilder.getFeatures(data2fp, b2Transform_zero, WorldBuilder::PARTITION));
	// printf("got features =%i\n", worldBuilder.world_objects.size());	
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d= now- endTime; //in seconds
	float duration=abs(float(d.count())/1000); //express in seconds
	// printf("built wolrd in %f\n", duration);
	explore_plan(world);
	worldBuilder.resetBodies();
	return 1;
}

void AttentiveConfigurator::resetPhi(TransitionSystem&g){
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		g[*vi].resetVisited();
		g[*vi].options.clear();
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
		std::vector <edgeDescriptor> in=gt::inEdges(g, v, UNDEFINED);
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
			else if (v==movingVertex){
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
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	Direction direction=currentTask.get_direction();
	std::vector <vertexDescriptor> priorityQueue = {v}, evaluationQueue, plan_prov=plan;
	std::set <vertexDescriptor> closed;
	Task t;
	b2Transform start= b2Transform_zero, shift=b2Transform_zero, shift_start=shift;
	EndedResult er;
	do{
		v=bestNext;
		closed.emplace(*priorityQueue.begin().base());
		priorityQueue.erase(priorityQueue.begin());
		er = controlGoal.checkEnded(g[v], t.get_direction());
		applyTransitionMatrix(g, v, direction, er.ended, v, plan_prov);
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0_exp=v;
			std::vector <Direction> options=g[v0_exp].options;
			while (!options.empty()){
				options.erase(options.begin());
				v0=v0_exp; //node being expanded
				v1 =v0; //frontier
				std::vector <vertexDescriptor> propagated;
				do {
					//task_to_sim_setup
				start=g[v0].endPose +shift;
				Disturbance Di=getDisturbance(g, v0, w, g[v0].options[0], start);
				t = Task(Di, g[v0].options[0], start, true);//need to update end crit
				std::pair <State, Edge> sk(State(start, Di, g[v0].options[0]), Edge());
				adjust_simulated_task(v0, g, &t);
				worldBuilder.buildWorld(w, t.getStart(), t.get_direction(), t.get_disturbance(), 0.15, WorldBuilder::PARTITION); //was g[v].endPose
				//end setup
				simResult sim=simulate(t, w); //sk.first, g[v0], 
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.it_observed=iteration;
				er  = Planner::estimateCost(sk.first, g[v0].endPose, sk.first.direction,controlGoal);
				StateDifference sd;
				std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(sk.first, g, g[v0].ID, t.get_direction(), StateMatcher::MATCH_TYPE::ABSTRACT, &sd);		//, closest_match	
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher.match_equal(match.first,StateMatcher::MATCH_TYPE::ABSTRACT)){
					g[v0].options.erase(g[v0].options.begin());
					v1=match.second; //frontier
						edge= gt::add_edge(v0, v1, g, iteration, t.get_direction()); //assumes edge added
					if (edge.second){
						g[edge.first]=sk.second; //doesn't update motorstep
					}
					if (currentTask.get_change()){
						std::vector <vertexDescriptor> task_vertices=gt::task_vertices(v1, g, iteration, currentVertex);
						vertexDescriptor task_start= task_vertices[0];
						if (plan_prov.empty()){
							bool finished=false, been=matcher.match_equal(match.first, StateMatcher::ABSTRACT); //(match.first==StateMatcher::DISTURBANCE); //ADD representation of task but shifted
							//shift here?
							Task controlGoal_adjusted= controlGoal;
							shift_start= b2MulT(b2MulT(sk.first.start, controlGoal.getStart()), g[task_start].start);
							math::applyAffineTrans(-shift_start, &controlGoal_adjusted); //as start
							boost::remove_edge(edge.first, g);
							edge= gt::add_edge(v0, task_start, g, iteration, g[edge.first.m_target].direction);
							ExecutionInfo info=package_info(TransitionSystem::null_vertex(), been);
							info.overarchingGoal(controlGoal_adjusted); 
							auto plan_tmp=planner->plan(g, v, info, &finished); //not v but task start
							//printf("out of explore planner\n");
							bool filler=0;
							if (finished){
								plan_prov=plan_tmp;
								if (plan_prov.empty()){ // task_start==currentVertex in\tead of pv empty
									//printf("inserting current vertex\n");
									plan_prov.insert(plan_prov.begin(), task_start);
								}
								if (t.get_direction()== g[task_start].direction){
									g[v0].options.clear();
								}
								else{
									g[v0].options={g[task_start].direction};
								}
							}
						}
						if (plan.empty() && g[task_start].options.empty() && g[v].options.empty()){
							shift_states(g, task_vertices, shift_start);
						}
					}
					
				}
				else{
					auto out_expected=gt::outEdges(g, v0, t.get_direction());
					edge= add_vertex_now(v0, v1,g,sk.first.Di, sk.second); //addVertex
					// //g[edge.first.m_target].label=sk.first.label; //new edge, valid
					// if (!out_expected.empty()){
					// 	vertexDescriptor exp=out_expected[0].m_target;
					// 	StateDifference sd_exp(g[v1], g[exp]);
					// 	printf("thought it'd be vertex %i , end pose:", exp );
					// }
					// printf("added vertex!");
					// debug::print_state_difference(sd, match.second, v1);
					shift=b2Transform_zero;
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, iteration);
					gt::adjustProbability(g, edge.first); //new_edge to allow to adjust prob if the sim state has been previously ecountered and split
				}
				applyTransitionMatrix(g, v1, t.get_direction(), er.ended, v0, plan_prov);
				g[v1].phi=Planner::evaluationFunction(er, v1, plan_prov);
				propagateD(v1, v0, g,&propagated, &closed); //og v1 v0
				v0_exp=v0;
				options=g[v0_exp].options;
				v0=v1;						
			}while(t.get_direction() !=DEFAULT & int(g[v0].options.size())!=0);
		evaluationQueue.push_back(v1);
		}
	}
	backtrack(evaluationQueue, priorityQueue, closed, g, plan_prov);
	bestNext=priorityQueue[0];
	//printf("best=%i end", bestNext);
	// debug::print_pose(g[bestNext].endPose);
	std::vector <edgeDescriptor> best_in_edges= gt::inEdges(g,bestNext);
	if (best_in_edges.empty()){
		direction=currentTask.get_direction();
	}
	else{
		direction = g[bestNext].direction;
		g[best_in_edges[0]].it_observed=iteration;
	}
}while(g[bestNext].options.size()>0 && !er.ended);
// printf("finished exploring, plan =%i\n", plan_prov.size());
return plan_prov;
}

std::vector <vertexDescriptor> AttentiveConfigurator::splitTask( vertexDescriptor v, TransitionSystem& g, Direction d, vertexDescriptor src){
	std::vector <vertexDescriptor> split={v};
	auto first_edge=boost::edge(src, v, g); //assumes exists

	if (gt::check_edge_direction(first_edge, g, RIGHT)|| gt::check_edge_direction(first_edge, g, LEFT)){ //d
		return split;
	}
	if (g[v].outcome != simResult::crashed){
		return split;
	}
	if (auto ie=gt::inEdges(g, src, DEFAULT), stop_edges=gt::inEdges(g, src, STOP); !ie.empty()|| !stop_edges.empty()){
		split.insert(split.begin(), src);
		g[src].outcome=simResult::safeForNow;
	}
	vertexDescriptor v1=v;
	float nNodes = g[v].distance()/simulationStep, og_phi=g[v].phi;
	b2Transform endPose = g[v].endPose;
	Task::Action a;
	a.init(d);
	while(nNodes>1){
		State s_tmp=State(g[v]);
		if(nNodes >1){
			b2Vec2 step_v(simulationStep*endPose.q.c, simulationStep*endPose.q.s);
			s_tmp.endPose=g[v].start+b2Transform(step_v, b2Rot(0));
			std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(s_tmp, g, NULL, d);
			if (match.first!=StateMatcher::_TRUE){
				g[v].options = {d};
				g[v].endPose=s_tmp.endPose;
				g[v].Dn=s_tmp.Dn;
				g[first_edge.first].step= gt::distanceToSimStep(g[v].distance(), a.getLinearSpeed());
				first_edge=add_vertex_retro(v, v1,g); 
				g[v1].Di=g[v].Di;
				g[v1].start=g[v].endPose;
				g[v].phi=NAIVE_PHI;
				g[v1].direction=d;
				g[v].outcome=simResult::safeForNow;
			}
			else{
				v1=match.second;
			}
			split.push_back(v1);
			nNodes--;
		}
		if (nNodes<=1){
			s_tmp.endPose=endPose;
			std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(s_tmp, g, NULL, d);
			if (match.first!=StateMatcher::_TRUE || match.second==v){
				g[v1].endPose = endPose;
				g[first_edge.first].step= gt::distanceToSimStep(g[v1].distance(), a.getLinearSpeed());	
				g[v1].outcome=simResult::crashed;	
				g[v1].phi=og_phi;
			}

		}

		v=v1;

	}
	return split;
}


void AttentiveConfigurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, const std::set<vertexDescriptor>& closed, TransitionSystem&g, std::vector <vertexDescriptor>& plan_prov){
	for (vertexDescriptor v:evaluation_q){
		std::pair<bool, edgeDescriptor> ep(false, edgeDescriptor());
		std::vector <vertexDescriptor> split = gt::task_vertices(v, g, iteration, currentVertex, &ep); 
		Direction direction= g[ep.second.m_target].direction;
		if (split.size()<2){
			split =splitTask(v, g, DEFAULT, ep.second.m_source);
		}
		for (int i=0; i<split.size(); i++){ //
			vertexDescriptor split_v=split[i], src=TransitionSystem::null_vertex();
			if (i<1){
				auto ep=gt::getMostLikely(g, gt::inEdges(g, split_v), iteration);
				if (ep.first){
					src=ep.second.m_source;
				}
			}
			else{
				src=split[i-1];
			}
			EndedResult local_er=Planner::estimateCost(g[split_v],g[split_v].start,direction, controlGoal);
			g[split_v].phi=Planner::evaluationFunction(local_er, split_v, plan_prov);
			applyTransitionMatrix(g, split_v, direction, local_er.ended,src, plan_prov);
			addToPriorityQueue(split_v, priority_q, g, closed);
			src=split_v;
		}
	}
	evaluation_q.clear();

}

void AttentiveConfigurator::propagateD(vertexDescriptor v1, vertexDescriptor v0,TransitionSystem&g, std::vector<vertexDescriptor>*propagated, std::set <vertexDescriptor>*closed,StateMatcher::MATCH_TYPE match){
	if (g[v1].outcome == simResult::successful){
		return;
	}
	vertexDescriptor p=TransitionSystem::null_vertex();
	std::pair <edgeDescriptor, bool> ep= boost::edge(v0, v1, g);
	Disturbance dist = g[v1].Dn;
	if (!ep.second){
		return;
	}
	Direction dir= g[v1].direction;
	bool same_Di=g[ep.first.m_source].Di==g[ep.first.m_target].Di;
	ep.first= *(boost::in_edges(ep.first.m_source, g).first);
	ep.second= boost::edge(ep.first.m_source, ep.first.m_target, g).second;
	bool same_direction=gt::check_edge_direction(ep, g, dir) ||( (gt::check_edge_direction(ep, g, STOP))&& dir==DEFAULT) ;
	if (same_direction&& same_Di && g[ep.first.m_target].Dn.getAffIndex()==NONE){
 			g[ep.first.m_target].Dn = dist; //was target
 	}
	if (v1==currentVertex){
		g[v0].outcome=simResult::safeForNow;
	}
	return;
}








void Configurator::printPlan(std::vector <vertexDescriptor>* p){

	std::vector <vertexDescriptor> _plan= plan;
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
			//c->track_task_execution();
			if (c->getIteration()>1){
				deltaPose= c->tracker->track((c->currentTask),c->ci->data2fp, c->worldBuilder.get_world_objects());
			}
			c->update_graph(c->transitionSystem, deltaPose);
			if (c->goal_changer!=NULL){
				if (( c->currentTask.get_change()& c->transitionSystem[c->currentVertex].direction!=STOP && c->plan.empty() && c->getIteration()>1)){
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


void AttentiveConfigurator::unexplored_transitions(TransitionSystem& g, const vertexDescriptor& v){
	std::vector <Direction> to_remove;
	for (int i=0; i<g[v].options.size(); i++){
		for (edgeDescriptor &e: gt::outEdges(g, v, g[v].options[i])){
			if (g[e].it_observed==iteration){ //g[e.m_target].visited()
				to_remove.push_back(g[e.m_target].direction);
			}
		}
	}
	for (Direction & d:to_remove){
		auto it=std::find(g[v].options.begin(), g[v].options.end(), d);
		if (it !=g[v].options.end()){
			g[v].options.erase(it);
		}
	}
}

void AttentiveConfigurator::transitionMatrix(State& state, Direction d, vertexDescriptor src){
	Task temp(controlGoal.get_disturbance(), DEFAULT, state.endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	if (state.outcome == simResult::safeForNow){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
				//in order, try the task which represents the reflex towards the goal
				if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
					state.options.push_back(temp.get_direction());
					state.options.push_back(getOppositeDirection(temp.get_direction()).second);
				}
				else{
					int random= rand();
					if (random%2==0){
						state.options = {LEFT, RIGHT};
					}
					else{
						state.options = {RIGHT, LEFT};
					}
				}
			}
	}
	else if (state.outcome==simResult::successful) { //will only enter if successful
		if (d== LEFT || d == RIGHT){
			state.options = {DEFAULT};
			if (src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.get_disturbance().pose().p)<0){
				state.options.push_back(d);
			}
		}
		else {
			if (src==TransitionSystem::null_vertex()){
				if (!currentTask.get_change()){
					state.options={currentTask.get_direction()};
				}
				else{
					state.options={DEFAULT, LEFT, RIGHT};
				}
				
			}
			else if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
				state.options.push_back(temp.get_direction());
				state.options.push_back(getOppositeDirection(temp.get_direction()).second);
				state.options.push_back(DEFAULT);
			}
			else{
				state.options={DEFAULT};
			}

		}

	}
}

void AttentiveConfigurator::applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov){
	if (!g[v0].options.empty()){
		return;
	}
	if (controlGoal.getEndCriteria().hasEnd()){
		if (ended){
			return;
		}
	}
	else if(round(g[v0].endPose.p.Length()*100)/100>=BOX2DRANGE){ // OR g[vd].totDs>4
		return;
	}
	if (src!=movingVertex  && uint(src)<(g.m_vertices.size()-1)&& v0!=movingVertex){ //src< v size is to check that src isn't a garbage value (was giving throuble with tests)
		auto e=boost::edge(src, v0, g); //not adding options to vertices which don't cover a distance unless they're current v
		if (e.second){
			if (g[e.first].step==0){
				return;
			}			
		}
	}
	std::vector <vertexDescriptor> full_plan=plan_prov;
	if (!currentTask.get_change()){
		full_plan.insert(full_plan.begin(), current_vertices.begin(), current_vertices.end());
	}
	if (v0==movingVertex || src==TransitionSystem::null_vertex()){
		transitionMatrix(g[v0], DEFAULT, TransitionSystem::null_vertex());	
	}
	else if (auto it =check_vector_for(full_plan, v0); it!=full_plan.end() && it!=(full_plan.end()-1)){
		auto e=boost::edge(src, v0, g);
		// if (!e.second){
		// 	printf("no edge wtf, %i -> %i\n", src, v0);
		// }
		gt::to_task_end(e.first, g, full_plan, it);
		if ((g[e.first.m_target].visited()&& g[e.first].it_observed<iteration)|| !g[e.first.m_target].visited()){ // 
			g[v0].options={g[e.first.m_target].direction};
		}
	}
	else{
		transitionMatrix(g[v0], d, src);
	}
	unexplored_transitions(g, v0);

}


void AttentiveConfigurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, TransitionSystem &g, const std::set <vertexDescriptor>& closed){
	if (g[v].outcome==simResult::crashed){
		return;
	}
	for (auto i =queue.begin(); i!=queue.end(); i++){
		bool expanded=0;
		auto found=closed.find(v); 
		if (g[v].phi <abs(g[*i].phi) && found==closed.end()){
			queue.insert(i, v);
			return;
		}
	}
	queue.push_back(v);
}


// void Configurator::addToPriorityQueue(Frontier f, std::vector<Frontier>& queue, TransitionSystem &g, vertexDescriptor goal){
// 	for (auto i =queue.begin(); i!=queue.end(); i++){
// 		if (g[f.first].phi <abs(g[(*i).first].phi)){
// 			queue.insert(i, f);
// 			return;
// 		}
// 	}
// 	queue.push_back(f);
// }


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

void AttentiveConfigurator::adjust_simulated_task(const vertexDescriptor &v, TransitionSystem &g, Task * t){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	if (t->get_direction()==currentTask.get_direction()){
		t->setEndCriteria(currentTask.getEndCriteria());
	}
	else if (t->get_direction()==getOppositeDirection(currentTask.get_direction()).second){
		t->setEndCriteria(Angle(M_PI-t->getEndCriteria().angle.get()));
	}
}



void AttentiveConfigurator::adjust_rw_task(const vertexDescriptor &v, TransitionSystem &g, Task * t, const b2Transform & deltaPose){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	if(!ep.second){ //no tgt	
		if (v==0){
			printf("edge doesn't exist");
		}
		return; //check until needs to be checked
	}
	// auto eb=boost::edge(currentEdge.m_source,currentEdge.m_target, transitionSystem);
	// int stepsTraversed= g[eb.first].step-currentTask.motorStep; //eb.first
	// float theta_exp=stepsTraversed*MOTOR_CALLBACK*currentTask.action.getOmega();
	// float theta_obs=theta_exp;//currentTask.correct.getError()-theta_exp;
	if (t->getAction().getOmega()!=0){
		float remainingAngle = t->getEndCriteria().angle.get()-abs(deltaPose.q.GetAngle());
	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
		// if (t->direction==getOppositeDirection(t->direction).second){
		// 	remainingAngle=M_PI-remainingAngle;
		// }
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(t->getAction().getLinearSpeed()>0){
		//step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
		t->setEndCriteria(Distance(t->getEndCriteria().distance.get()-deltaPose.p.Length()));
	}			// -estimated distance covered

}




	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> AttentiveConfigurator::findMatch(State s, TransitionSystem& g, State * src, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd){
	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex()), backup=result;
	auto vs= boost::vertices(g);
	float prob=0, sum=10000;
	//need to find best match too
	ComparePair comparePair;
	std::set <std::pair <vertexDescriptor, float>, ComparePair>others_set(comparePair);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=dir==Direction::UNDEFINED ||g[v].direction==dir;
		//make state representing a whole task, this is inefficient and when i have time should be susbtituted with subgraph
		State q= g[v];
			if (auto vertices=gt::task_vertices(v, g, iteration, currentVertex); vertices.size()>1){
				q.start=g[vertices[0]].start;
			}			
		if (v==currentVertex && !currentTask.get_change()){
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
		
		if (v!=movingVertex && boost::in_degree(v, g)>0 &&Tmatch ){ 
			if (condition){
				result.first= m;
				result.second=v;
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

	}

	// if (others==NULL){
	// 	return result;
	// }
	// printf("others not null\n");
	// for (auto vp:others_set){
	// 	others->push_back(vp.first);
	// }
	return result;
}



void AttentiveConfigurator::planPriority(TransitionSystem&g, vertexDescriptor v){
    for (vertexDescriptor p:plan){
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
		math::applyAffineTrans(shift_start, g[v]);
	}
}

vertexDescriptor AttentiveConfigurator::get_explore_start(TransitionSystem & g){
	if (g.m_vertices.size()==1){
		dummy_vertex(currentVertex);
		currentTask.set_change(true);
	}
	if (!plan.empty() || !currentTask.get_change()){ //
		return movingVertex;
	}
	else{
		return currentVertex;
	}
}

void AttentiveConfigurator::pre_explore(){
	//if (movingVertex!=currentVertex){
		boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
	//}	
		//transitionSystem[movingVertex].Di=currentTask.get_disturbance();
		transitionSystem[movingVertex].Di=transitionSystem[currentVertex].Di;

		transitionSystem[movingVertex].outcome=simResult::successful;
		movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
	// if (currentTask.get_change()){
	// 	transitionSystem[movingEdge].step=currentTask.getMotorStep();
	// }
}

std::vector <State> AttentiveConfigurator::output_plan(const std::vector <vertexDescriptor>& p, const TransitionSystem &g){
	std::vector <State> rho;
	for (const vertexDescriptor &v: p){
		rho.push_back(g[v]);
	}
	return rho;
}

void Configurator::estimate_current_vertex(){
	if(current_vertices.empty()){
		currentVertex=movingVertex;
		return;
	}
	if (current_vertices.size()==1){
		currentVertex=current_vertices[0];
		auto e=boost::add_edge(movingVertex, currentVertex, transitionSystem);
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
	//auto e=boost::add_edge(movingVertex, currentVertex, transitionSystem);
	//movingEdge=e.first;

}


void Configurator::change_task(){
	if (!currentTask.get_change()){
		return;
	}
	if (task_controller==NULL){
		throw std::invalid_argument("no controller, please add!");
	}
	printf("change!\n");
	task_controller->next_task(currentTask, controlGoal, transitionSystem, current_vertices, plan);
	transitionSystem[movingEdge].step=currentTask.getMotorStep();
	//printPlan();
	tracker->on_new_task(&currentTask);
	control->reset();
	control->getData(currentTask.action);
	return;
}

void Configurator::update_graph(TransitionSystem&g, const b2Transform & _deltaPose){
	math::applyAffineTrans(_deltaPose, g);
	math::applyAffineTrans(_deltaPose, &controlGoal);
}


void Configurator::adjust_goal_expectation(){
	if (controlGoal.getAffIndex()==PURSUE && !plan.empty()&&task_controller->get_disturbance().getAffIndex()!=NONE){
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
// 		math::applyAffineTrans(difference, &controlGoal);//update goal with ratio info
		debug::print_pose(controlGoal.disturbance.pose(), "goal after adjusting");
		printf("distance after adjusting %f\n", controlGoal.disturbance.pose().p.Length());
	}

}

void AttentiveConfigurator::explore_plan(b2World&world){
    pre_explore();
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi(transitionSystem);
    std::vector <vertexDescriptor> plan_tmp=explorer(src, transitionSystem, world);
    if (DEBUG){
        std::vector<vertexDescriptor> _plan=(plan);
        debug::graph_file(iteration, transitionSystem, controlGoal.get_disturbance(), _plan, currentVertex);
    }	
	try{
		ts_cleanup(transitionSystem, plan); //remove self-edge and singleton states
	}
	catch(...){}	
    if (plan_tmp.empty() && (!transitionSystem[currentVertex].visited() || currentTask.get_change())){ //currentv not visited means that it wasn't observed ()
        printf("no plan, searchign from %i\n", src);
        bool finished=false;
        ExecutionInfo info=package_info();
        plan_tmp= planner->plan(transitionSystem, currentVertex,info, &finished); //src
    }
    else{
        printf("recycled plan in explorer:\n");
    }
    plan=plan_tmp;
    printPlan(&plan);
}

void ReactiveConfigurator::explore_plan(b2World &world){
	if (transitionSystem.m_vertices.size()==1 && iteration<=1){
		movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingVertex].direction=DEFAULT;
		currentTask.getAction().init(transitionSystem[currentVertex].direction);
	}
	if (currentTask.getAction().getOmega()!=0 && currentTask.getMotorStep()<(transitionSystem[movingEdge].step)){
		return;
	}
	//adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
	worldBuilder.buildWorld(world, transitionSystem[movingVertex].start, currentTask.get_direction()); //was g[v].endPose
	Task t=currentTask;
	t.H(t.get_disturbance(), t.get_direction(), true);
	simResult result = simulate(t, world); //transitionSystem[currentVertex],transitionSystem[currentVertex],
	gt::fill(result, transitionSystem[currentVertex].ID, &transitionSystem[currentEdge]);
	
	//transitionSystem[currentVertex].Dn.set_affordance(as.affordance);
	currentTask.set_change(transitionSystem[currentVertex].outcome!=simResult::successful);
	if (currentTask.get_change()){
		printf("crashed\n");
	}
}
