#include "configurator.h"
#include <chrono>



// bool LIDAR_In::isReady(){
// 	return ready;
// }

void Configurator::dummy_vertex(vertexDescriptor src){
	vertexDescriptor prev_current=currentVertex;
	currentVertex=boost::add_vertex(transitionSystem);
	gt::fill(simResult(), &transitionSystem[currentVertex]);
	transitionSystem[currentVertex].nObs++;
	transitionSystem[currentVertex].Di=controlGoal.disturbance;
	currentTask=Task(Direction::STOP);
	movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
	currentEdge = boost::add_edge(src, currentVertex, transitionSystem).first;
	printf("dummy, current edge = %i, %i\n", src, currentVertex);
	transitionSystem[movingVertex].direction=STOP;
	transitionSystem[currentVertex].direction=STOP;
}

std::pair <edgeDescriptor, bool> Configurator::add_vertex_now(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Disturbance obs,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, g, edge, topDown);
	if (!g[v1].filled){
		g[v1].Di= obs;
	}
	return result;
}

std::pair <edgeDescriptor, bool> Configurator::add_vertex_retro(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Disturbance obs,Edge edge, bool topDown){
	std::pair<edgeDescriptor, bool> result=addVertex(src, v1, g, edge, topDown);
	g[v1].Di= g[src].Di;
	g[v1].Dn=g[src].Dn;
	return result;
}




bool Configurator::Spawner(){ 
	//PREPARE VECTORS TO RECEIVE DATA
	iteration++; //iteration set in getVelocity
	worldBuilder.iteration++;

	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>diff= now - previousTimeScan; //in seconds
	//timeElapsed=float(diff.count())/1000; //express in seconds
	previousTimeScan=now; //update the time of sampling

	// if (timerOff){
	// 	timeElapsed = .2;
	// }

	//CREATE BOX2D ENVIRONMENT
	b2Vec2 gravity = {0.0, 0.0};
	b2World world= b2World(gravity);
	char name[256];
	worldBuilder.world_objects=worldBuilder.getFeatures(data2fp, b2Transform_zero);
	explore_plan(world);
	worldBuilder.resetBodies();
	return 1;
}

void Configurator::resetPhi(TransitionSystem&g){
	auto vs=boost::vertices(g);
	for (auto vi=vs.first; vi!=vs.second; vi++){
		g[*vi].resetVisited();
		g[*vi].options.clear();
	}
}



std::pair <bool, Direction> Configurator::getOppositeDirection(Direction d){
	std::pair <bool, Direction> result(false, DEFAULT);
		switch (d){
		case Direction::LEFT: result.first = true; result.second = RIGHT;break;
		case Direction::RIGHT: result.first = true; result.second = LEFT;break;
		default:
		break;
	}
	return result;
}
Disturbance Configurator::getDisturbance(TransitionSystem&g, const  vertexDescriptor& v, b2World & world, const Direction& dir, const b2Transform& start){
	if (!g[v].Dn.isValid() ){
		std::vector <edgeDescriptor> in=gt::inEdges(g, v, UNDEFINED);
		std::vector <edgeDescriptor> out=gt::outEdges(g, v, UNDEFINED);
		std::pair <bool,edgeDescriptor> visited= gt::visitedEdge(in,g, v);
			if (visited.first ||out.empty()){
				if (g[v].Di.isValid() && g[v].Di.getAffIndex()==AVOID && g[visited.second.m_target].direction!=dir){
					Task task(g[v].Di, DEFAULT, g[v].endPose, true);
					Robot robot(&world);
					robot.body->SetTransform(task.start.p, task.start.q.GetAngle());
					b2AABB box =worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
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
				return controlGoal.disturbance;
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
	worldBuilder.bodies++;
	robot.body->SetTransform(t.start.p, t.start.q.GetAngle());
	b2AABB sensor_aabb=worldBuilder.makeRobotSensor(robot.body, &controlGoal.disturbance);
	result =t.bumping_that(w, iteration, robot.body, remaining); //default start from 0
	printf("before cleanup\n");
	//approximate angle to avoid stupid rounding errors
	float approximated_angle=approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode);
	result.endPose.q.Set(approximated_angle);
	return result;
	}


std::vector<vertexDescriptor> Configurator::explorer(vertexDescriptor v, TransitionSystem& g, b2World & w){
	vertexDescriptor v1=v, v0=v, bestNext=v, v0_exp=v;
	Direction direction=currentTask.direction;
	std::vector <vertexDescriptor> priorityQueue = {v}, evaluationQueue, plan_prov=plan;
	std::set <vertexDescriptor> closed;
	Task t;
	b2Transform start= b2Transform_zero, shift=b2Transform_zero, shift_start=shift;
	EndedResult er;
	debug::print_pose(controlGoal.disturbance.pose(), "Goal at:");
	do{
		v=bestNext;
		closed.emplace(*priorityQueue.begin().base());
		priorityQueue.erase(priorityQueue.begin());
		er = controlGoal.checkEnded(g[v], t.direction);
		applyTransitionMatrix(g, v, direction, er.ended, v, plan_prov);
		//printf("v=%i options =%in", v, g[v].options.size());		
		for (Direction d: g[v].options){ //add and evaluate all vertices
			v0_exp=v;
			std::vector <Direction> options=g[v0_exp].options;
			while (!options.empty()){
				options.erase(options.begin());
				v0=v0_exp; //node being expanded
				v1 =v0; //frontier
				std::vector <vertexDescriptor> propagated;
				do {
				start=g[v0].endPose +shift;
				//debug::print_pose(start, "siulation start");
				Disturbance Di=getDisturbance(g, v0, w, g[v0].options[0], start);
				t = Task(Di, g[v0].options[0], start, true);//need to update end crit
				std::pair <State, Edge> sk(State(start, Di, g[v0].options[0]), Edge());
				adjust_simulated_task(v0, g, &t);
				worldBuilder.buildWorld(w, t.start, t.direction, t.disturbance, 0.15, WorldBuilder::PARTITION); //was g[v].endPose
				simResult sim=simulate(t, w); //sk.first, g[v0], 
//				worldBuilder.world_cleanup(w);
				if (v==0 && sim.resultCode==sim.crashed){
					printf("IM GONNA CRASH!!!! at");
					debug::print_pose(sim.collision.pose());
				}
				gt::fill(sim, &sk.first, &sk.second); //find simulation result
				sk.second.it_observed=iteration;
				er  = estimateCost(sk.first, g[v0].endPose, sk.first.direction);
				//State * source=NULL;
				StateDifference sd;
				std::pair<StateMatcher::MATCH_TYPE, vertexDescriptor> match=findMatch(sk.first, g, g[v0].ID, t.direction, StateMatcher::MATCH_TYPE::ABSTRACT, &sd);		//, closest_match	
				std::pair <edgeDescriptor, bool> edge(edgeDescriptor(), false); //, new_edge(edgeDescriptor(TransitionSystem::null_vertex(), TransitionSystem::null_vertex(), NULL), false);
				if (matcher.match_equal(match.first,StateMatcher::MATCH_TYPE::ABSTRACT)){
					g[v0].options.erase(g[v0].options.begin());
					v1=match.second; //frontier
				//	printf("match with %i\n", v1);
						edge= gt::add_edge(v0, v1, g, iteration, t.direction); //assumes edge added
					if (edge.second){
						//printf("added edge: %i -> %i, step=%i\n", v0, v1, sk.second.step);
						g[edge.first]=sk.second; //doesn't update motorstep
					}
					if (currentTask.change){
						std::vector <vertexDescriptor> task_vertices=gt::task_vertices(v1, g, iteration, currentVertex);
						vertexDescriptor task_start= task_vertices[0];
						if (plan_prov.empty()){
							bool finished=false, been=matcher.match_equal(match.first, StateMatcher::ABSTRACT); //(match.first==StateMatcher::DISTURBANCE); //ADD representation of task but shifted
							//shift here?
							Task controlGoal_adjusted= controlGoal;
							shift_start= b2MulT(b2MulT(sk.first.start, controlGoal.start), g[task_start].start);
							math::applyAffineTrans(shift_start, &controlGoal_adjusted); //as start
							boost::remove_edge(edge.first, g);
							edge= gt::add_edge(v0, task_start, g, iteration, g[edge.first.m_target].direction);
							auto plan_tmp=planner(g, v, TransitionSystem::null_vertex(), been, &controlGoal_adjusted, &finished); //not v but task start
							printf("out of explore planner\n");
							bool filler=0;
							// if (match.second!=v){
							// 	shift= b2MulT(g[task_start].start, start);
							// }
							if (finished){
								plan_prov=plan_tmp;
								if (plan_prov.empty()){ // task_start==currentVertex in\tead of pv empty
									printf("inserting current vertex\n");
									plan_prov.insert(plan_prov.begin(), task_start);
								}
								if (t.direction== g[task_start].direction){
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
					auto out_expected=gt::outEdges(g, v0, t.direction);
					edge= add_vertex_now(v0, v1,g,sk.first.Di, sk.second); //addVertex
					//g[edge.first.m_target].label=sk.first.label; //new edge, valid
					if (!out_expected.empty()){
						vertexDescriptor exp=out_expected[0].m_target;
						StateDifference sd_exp(g[v1], g[exp]);
						printf("thought it'd be vertex %i , end pose:", exp );
						// FILE * err_file=fopen("/tmp/err_file.txt", "a+");
						// char * sd_di=  debug::print_pose(sd_exp.Di.pose), *sd_dn=debug::print_pose(sd_exp.Dn.pose);
						// fprintf(err_file, "%s\t%s",sd_di, sd_dn );
						// fclose(err_file);
						//debug::print_pose(g[exp].endPose);
					}
					printf("added vertex!");
					debug::print_state_difference(sd, match.second, v1);
					//auto d_print=dirmap.find(t.direction);
					//printf("added v %i to %i, direction %s", v1, v0, (*d_print).second);
					shift=b2Transform_zero;
				}
				if(edge.second){
					gt::set(edge.first, sk, g, v1==currentVertex, iteration);
					gt::adjustProbability(g, edge.first); //new_edge to allow to adjust prob if the sim state has been previously ecountered and split
				}
				applyTransitionMatrix(g, v1, t.direction, er.ended, v0, plan_prov);
				g[v1].phi=evaluationFunction(er, v1, plan_prov);
				propagateD(v1, v0, g,&propagated, &closed); //og v1 v0
				v0_exp=v0;
				options=g[v0_exp].options;
				v0=v1;						
			}while(t.direction !=DEFAULT & int(g[v0].options.size())!=0);
		evaluationQueue.push_back(v1);
		}
	}
	backtrack(evaluationQueue, priorityQueue, closed, g, plan_prov);
	bestNext=priorityQueue[0];
	//printf("best=%i end", bestNext);
	// debug::print_pose(g[bestNext].endPose);
	std::vector <edgeDescriptor> best_in_edges= gt::inEdges(g,bestNext);
	if (best_in_edges.empty()){
		direction=currentTask.direction;
	}
	else{
		direction = g[bestNext].direction;
		g[best_in_edges[0]].it_observed=iteration;
	}
}while(g[bestNext].options.size()>0 && !er.ended);
//printf("finished exploring, plan =%i\n", plan_prov.size());
return plan_prov;
}

std::vector <vertexDescriptor> Configurator::splitTask( vertexDescriptor v, TransitionSystem& g, Direction d, vertexDescriptor src){
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
				first_edge=add_vertex_retro(v, v1,g, g[v].Dn); //passing on the disturbance
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


void Configurator::backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, const std::set<vertexDescriptor>& closed, TransitionSystem&g, std::vector <vertexDescriptor>& plan_prov){
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
			EndedResult local_er=estimateCost(g[split_v],g[split_v].start, direction);
			g[split_v].phi=evaluationFunction(local_er, split_v, plan_prov);
			applyTransitionMatrix(g, split_v, direction, local_er.ended,src, plan_prov);
			addToPriorityQueue(split_v, priority_q, g, closed);
			src=split_v;
		}
	}
	evaluation_q.clear();

}

void Configurator::propagateD(vertexDescriptor v1, vertexDescriptor v0,TransitionSystem&g, std::vector<vertexDescriptor>*propagated, std::set <vertexDescriptor>*closed,StateMatcher::MATCH_TYPE match){
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

std::vector <vertexDescriptor> Configurator::planner( TransitionSystem& g, vertexDescriptor src, vertexDescriptor goal, bool been, const Task* custom_ctrl_goal, bool *finished){
	std::vector<std::vector<vertexDescriptor>> paths;
	paths.push_back(std::vector<vertexDescriptor>()={src});
	std::vector <Frontier> frontier_v;
	bool run=true, _finished=false;
	std::vector <Frontier> priorityQueue={Frontier(src, std::vector<vertexDescriptor>())};
	Task overarching_goal;
	if (NULL==custom_ctrl_goal){
		overarching_goal=controlGoal;
	}
	else{
		overarching_goal=*custom_ctrl_goal;
	}
	int no_out=0;
	std::vector <vertexDescriptor> add;
	std::vector<std::vector<vertexDescriptor>>::reverse_iterator path= paths.rbegin();
	vertexDescriptor path_end=src;
	auto start_time=std::chrono::high_resolution_clock::now();
	do{
		frontier_v=frontierVertices(src, g, DEFAULT, been); // get next default tasks (plus non-default connecting tasks)
		priorityQueue.erase(priorityQueue.begin());
		for (Frontier f: frontier_v){ //add to priority queue
			//planPriority(g, f.first);
			addToPriorityQueue(f, priorityQueue, g);
		}
		if (!priorityQueue.empty()){
			src=priorityQueue.begin()->first; //lowest phi vertex
			add=std::vector <vertexDescriptor>(priorityQueue.begin()->second.begin(), priorityQueue.begin()->second.end());//lowest phi frontier
			add.push_back(src);
			Planner::path2add2(path, add, paths, g); //find path to add frontier (add) to
			for (vertexDescriptor c:add){
				g[c].label=VERTEX_LABEL::UNLABELED;
				path->push_back(c);	
				path_end=c;			
			}
		}
		_finished=overarching_goal.checkEnded(g[path_end].endPose, UNDEFINED, true).ended;
		if (NULL!=finished){
			*finished=_finished;
		}
		if (_finished){
			goal=path_end;
		}
	}while(!priorityQueue.empty() && (path_end!=goal && !(_finished)));
	return Planner::best_path(paths, goal, currentVertex, currentTask.change, g);
}




EndedResult Configurator::estimateCost(State &state, b2Transform start, Direction d){
	EndedResult er = controlGoal.checkEnded(state);
	Task t(state.Dn, d, start);
	er.cost += t.checkEnded(state.endPose).estimatedCost;
	if (state.outcome==simResult::crashed){
		er.cost+=2;
	}
	return er;
}


float Configurator::evaluationFunction(EndedResult er,  const vertexDescriptor& v, std::vector<vertexDescriptor>& p){ 
	float result=(abs(er.estimatedCost)+abs(er.cost))/2;
	if (auto it=check_vector_for(p, v); it!=p.end()){
		result-=0.1;
	}
	return result; //normalised to 1
}



void Configurator::printPlan(std::vector <vertexDescriptor>* p){
	std::vector <vertexDescriptor> plan= *p;
	vertexDescriptor pre=currentVertex;
	printf("PLAN:");
	for (vertexDescriptor v: plan){
		std::pair <edgeDescriptor, bool> edge=boost::edge(pre, v, transitionSystem);
		if (!edge.second){
			printf("no edge: %i-> %i", edge.first.m_source, edge.first.m_target);
		}
		else{
			auto a=dirmap.find(transitionSystem[v].direction);
			printf("%i, %s, (%i steps) ", edge.first.m_target, (*a).second, transitionSystem[edge.first].step);
		}
		pre=edge.first.m_target;
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
		if (c->ci->isReady()){
			printf("ci ready\n");
			c->ci->setReady(false);
			c->data2fp= CoordinateContainer(c->ci->data2fp);
			c->Spawner();
			printf("graph size=%i\n", c->transitionSystem.m_vertices.size());
			c->track_task_execution();
		}
		if (( c->getTask()->change& c->transitionSystem[c->currentVertex].direction!=STOP && c->plan.empty() && c->getIteration()>1)){
			printf("change goal");
			c->goal_changer->change_goal(&c->controlGoal);
		}	
		if (!PLANNING){
			printf("no planning!");
		}	
		c->change_task();

	}

}


void Configurator::unexplored_transitions(TransitionSystem& g, const vertexDescriptor& v){
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

void Configurator::transitionMatrix(State& state, Direction d, vertexDescriptor src){
	Task temp(controlGoal.disturbance, DEFAULT, state.endPose); //reflex to disturbance
	srand(unsigned(time(NULL)));
	if (state.outcome == simResult::safeForNow){ //accounts for simulation also being safe for now
		if (d ==DEFAULT ||d==STOP){
				//in order, try the task which represents the reflex towards the goal
				if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
					state.options.push_back(temp.direction);
					state.options.push_back(getOppositeDirection(temp.direction).second);
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
			if (src==currentVertex && controlGoal.getAffIndex()==PURSUE && SignedVectorLength(controlGoal.disturbance.pose().p)<0){
				state.options.push_back(d);
			}
		}
		else {
			if (src==TransitionSystem::null_vertex()){
				if (!currentTask.change){
					state.options={currentTask.direction};
				}
				else{
					state.options={DEFAULT, LEFT, RIGHT};
				}
				
			}
			else if (temp.getAction().getOmega()!=0){ //if the task chosen is a turning task
				state.options.push_back(temp.direction);
				state.options.push_back(getOppositeDirection(temp.direction).second);
				state.options.push_back(DEFAULT);
			}
			else{
				state.options={DEFAULT};
			}

		}

	}
}

void Configurator::applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov){
	if (!g[v0].options.empty()){
		return;
	}
	if (controlGoal.endCriteria.hasEnd()){
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
	if (!currentTask.change){
		full_plan.insert(full_plan.begin(), current_vertices.begin(), current_vertices.end());
	}
	if (v0==movingVertex || src==TransitionSystem::null_vertex()){
		transitionMatrix(g[v0], DEFAULT, TransitionSystem::null_vertex());	
	}
	else if (auto it =check_vector_for(full_plan, v0); it!=full_plan.end() && it!=(full_plan.end()-1)){
		auto e=boost::edge(src, v0, g);
		if (!e.second){
			printf("no edge wtf, %i -> %i\n", src, v0);
		}
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


void Configurator::addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, TransitionSystem &g, const std::set <vertexDescriptor>& closed){
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


void Configurator::addToPriorityQueue(Frontier f, std::vector<Frontier>& queue, TransitionSystem &g, vertexDescriptor goal){
	for (auto i =queue.begin(); i!=queue.end(); i++){
		if (g[f.first].phi <abs(g[(*i).first].phi)){
			queue.insert(i, f);
			return;
		}
	}
	queue.push_back(f);
}


std::pair <edgeDescriptor, bool> Configurator::maxProbability(std::vector<edgeDescriptor> ev, TransitionSystem& g){
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

void Configurator::adjust_simulated_task(const vertexDescriptor &v, TransitionSystem &g, Task * t){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, g);

	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	if (t->direction==currentTask.direction){
		t->endCriteria=currentTask.endCriteria;
	}
	else if (t->direction==getOppositeDirection(currentTask.direction).second){
		t->setEndCriteria(Angle(M_PI-t->endCriteria.angle.get()));
	}
}



void Configurator::adjust_rw_task(const vertexDescriptor &v, TransitionSystem &g, Task * t, const b2Transform & deltaPose){
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
		float remainingAngle = t->endCriteria.angle.get()-abs(deltaPose.q.GetAngle());
	//	printf("step =%i/%i, remaining angle=%f\n", currentTask.motorStep, transitionSystem[currentEdge].step,remainingAngle);
		// if (t->direction==getOppositeDirection(t->direction).second){
		// 	remainingAngle=M_PI-remainingAngle;
		// }
		t->setEndCriteria(Angle(remainingAngle));
	}
	if(t->getAction().getLinearSpeed()>0){
		//step-= (stepsTraversed*MOTOR_CALLBACK)*currentTask.action.getLinearSpeed();
		t->setEndCriteria(Distance(t->endCriteria.distance.get()-deltaPose.p.Length()));
	}			// -estimated distance covered

}

std::vector <Frontier> Configurator::frontierVertices(vertexDescriptor v, TransitionSystem& g, Direction d, bool been){
	std::vector <Frontier> result;
	std::pair<edgeDescriptor, bool> ep=boost::edge(movingVertex, v, g); 
	vertexDescriptor v0=v, v1=v, v0_exp;
	//do{
		if ((controlGoal.disturbance.getPosition()-g[v].endPose.p).Length() >= DISTANCE_ERROR_TOLERANCE){
			auto es=boost::out_edges(v, g);
			for (auto ei=es.first; ei!=es.second; ei++){
			std::vector <vertexDescriptor>connecting;
			auto ei2=ei, ei3=ei;
			auto es2=boost::out_edges((*ei).m_target, g);
			auto es3=es2;
			std::vector <vertexDescriptor>connecting2;
			NotSelfEdge not_self_edge(&g);
			do {
				if ((g[(*ei3).m_target].visited() || been)&& not_self_edge(*ei3)){ //(*ei3).m_source!=(*ei3).m_target
					if (!g[(*ei3).m_target].visited()){
						EndedResult er = estimateCost(g[(*ei3).m_target], g[(*ei3).m_source].endPose, g[(*ei3).m_target].direction);
						g[(*ei3).m_target].phi=evaluationFunction(er, (*ei3).m_target, plan);
					}
					if (g[(*ei3).m_target].direction==d){
						Frontier f;
						f.first= (*ei3).m_target;
						f.second=connecting2;
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
							es3=es2;						}
					}					
				}
				//printf("is stuck, ei3=%i ->%i\n", (*ei).m_source, (*ei).m_target);
			}while (ei3!=es3.second);
	}
	}

	return result;
}



	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> Configurator::findMatch(State s, TransitionSystem& g, State * src, Direction dir, StateMatcher::MATCH_TYPE match_type, StateDifference * _sd){
	std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> result(StateMatcher::MATCH_TYPE::_FALSE, TransitionSystem::null_vertex()), backup=result;
	auto vs= boost::vertices(g);
	float prob=0, sum=10000;
	//need to find best match too
	ComparePair comparePair;
	std::set <std::pair <vertexDescriptor, float>, ComparePair>others_set(comparePair);
	for (auto vi=vs.first; vi!= vs.second; vi++){
		vertexDescriptor v=*vi;
		bool Tmatch=dir==Direction::UNDEFINED ||g[v].direction==dir;
		//std::vector <edgeDescriptor> ie=gt::inEdges(g, v, dir);
		//Tmatch=!ie.empty()||dir==Direction::UNDEFINED;
		//make state representing a whole task, this is inefficient and when i have time should be susbtituted with subgraph
		State q= g[v];
//		if (wholeTask){
			if (auto vertices=gt::task_vertices(v, g, iteration, currentVertex); vertices.size()>1){
				q.start=g[vertices[0]].start;
			}			
//		}
		if (v==currentVertex && !currentTask.change){
			q.start=b2Transform_zero;
		}
		StateDifference sd(s, q);
		bool condition=0;
		StateMatcher::MATCH_TYPE m=StateMatcher::_FALSE;
		float sum_tmp=fabs(sd.get_sum(match_type));
		//if (!relax){
			m=matcher.isMatch(sd, s.endPose.p.Length());
			condition=matcher.match_equal(m, match_type);
		//}
		//else{
			//condition= sum_tmp<sum;
		//}
		
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
			// }
			// else{
			// 	result.first=match_type;
			// }
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



void Configurator::planPriority(TransitionSystem&g, vertexDescriptor v){
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


void Configurator::ts_cleanup(TransitionSystem & g, std::vector <vertexDescriptor>& p){
	Connected connected(&g);
	ViableEdge ke(&g);
	FilteredTS fts(g, ke, connected); //boost::keep_all()
	TransitionSystem tmp;
	boost::copy_graph(fts, tmp);
	g.clear();
	g.swap(tmp);		
}
 
void Configurator::shift_states(TransitionSystem & g, const std::vector<vertexDescriptor>& p, const b2Transform & shift_start){
	if (p.empty()){
		return;
	}
	for (const vertexDescriptor &v:p){
		math::applyAffineTrans(shift_start, g[v]);
	}
}

vertexDescriptor Configurator::get_explore_start(TransitionSystem & g){
	if (g.m_vertices.size()==1){
		dummy_vertex(currentVertex);
		currentTask.change=1;
	}
	if (!plan.empty() || !currentTask.change){ //
		return movingVertex;
	}
	else{
		return currentVertex;
	}
}

void Configurator::pre_explore(TransitionSystem & g, const std::vector<vertexDescriptor>& p, const bool& change){
	if (!change){
		boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
	}
	else{
	//	transitionSystem[movingVertex].Di=transitionSystem[currentVertex].Di;
		transitionSystem[movingVertex].Di=currentTask.disturbance;

		transitionSystem[movingVertex].outcome=simResult::successful;
		movingEdge=boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		boost::remove_out_edge_if(movingVertex, is_not_v(currentVertex), transitionSystem);
		std::pair<edgeDescriptor, bool> ep(edgeDescriptor(), false);
		if (!p.empty()){
			ep=boost::edge(currentVertex, p[0], g);
			//transitionSystem[curre].direction=transitionSystem[ep.first].direction;
		}
		transitionSystem[movingEdge].step=currentTask.motorStep;
	}
}

std::vector <State> Configurator::output_plan(const std::vector <vertexDescriptor>& p, const TransitionSystem &g){
	std::vector <State> rho;
	for (const vertexDescriptor &v: p){
		rho.push_back(g[v]);
	}
	return rho;
}

vertexDescriptor Configurator::estimate_current_vertex(TransitionSystem& g, Task& currentTask, vertexDescriptor currentVertex){
	vertexDescriptor task_start;
	try {
		task_start=current_vertices.at(0);
	}
	catch(const std::out_of_range& oor){
		printf("current vertices empty\n");
		return currentVertex;
	}
	b2Transform Di_distance=currentTask.from_Di(), v_from_D=b2Transform_zero;

	float sum=10000;
	StateMatcher matcher;
	for (vertexDescriptor & v:current_vertices){
		if ((g[task_start].Dn.getAffIndex()==AVOID && currentTask.disturbance.getAffIndex()==PURSUE)){
			v_from_D=g[v].start_from_Dn();
		}
		else{
			v_from_D=g[v].start_from_Di();
		}
		b2Transform transform_diff=Di_distance-v_from_D;
		float sum_diff=fabs(transform_diff.p.x+transform_diff.p.y+transform_diff.q.GetAngle());
		if (sum_diff<sum){
			currentVertex=v;
			sum=sum_diff;
		}				
	}
	return currentVertex;

}

void Configurator::track_task_execution(){
	printf("task L=%f, R=%f\n", currentTask.action.L, currentTask.action.R);
	b2Transform deltaPose=worldBuilder.wb_bridger.get_transform(currentTask, data2fp, &currentTask.disturbance); //track using obstacle OR dead reckoning
	currentTask.endCriteria.adjust(deltaPose); //adjusting in task so system can be memoryless
	update_graph(transitionSystem, deltaPose, &currentTask, &controlGoal);
	bool ended=false;
	ended=currentTask.checkEnded(task_sensor, b2Transform_zero, worldBuilder.wb_bridger.get_tracked_disturbance()); //the sensor moves with the robot
	
	if(currentTask.motorStep==0 || ended){
		currentTask.change=1;
	}
	currentVertex=estimate_current_vertex(transitionSystem, currentTask,currentVertex);
}

void Configurator::change_task(){
	if (!currentTask.change){
		//printf("not changing\n");
		return;
	}
//	printf("change task planning =%i\n", PLANNING);
	if (PLANNING){
		if (plan.empty()){
			//printf("I DON'T KNOW WHAT TO DO NOW\n");
			currentTask=Task(controlGoal.disturbance, UNDEFINED);
			currentTask.action.L=0;
			currentTask.action.R=0;
			currentTask.change=1;
			return;
		}
		printf("changing\n");
		int i=to_task_end();
		try{
			if (i==0){
				throw (i);
			}
		}
		catch (int index){
			i++;
		}
		current_vertices=std::vector(plan.begin(), plan.begin()+i);
		printPlan(&plan);
		//printf("change task=%i, task step=%i\n", currentTask.change, currentTask.motorStep);
		currentTask = task_to_execute(plan, transitionSystem, i);	
		plan.erase(plan.begin(), plan.begin()+i);
		//set end criteria to adjust error??
		task_sensor=worldBuilder.sensor_box(Robot::get_vertices(),b2Transform_zero, &(controlGoal.disturbance));
	}
	else{
		if (transitionSystem[0].Dn.isValid()){
			currentTask= Task(transitionSystem[0].Dn, DEFAULT); //reactive
		}
		else{
			currentTask = Task(controlGoal.disturbance, DEFAULT); //reactive
		}
		currentTask.motorStep = motor_step(currentTask.getAction());
		printf("changed to %f\n", currentTask.action.getOmega());
	}
	worldBuilder.wb_bridger.set_tracked_disturbance(currentTask.disturbance);
	control->getData(currentTask.action);
	return;
}

void Configurator::update_graph(TransitionSystem&g, const b2Transform & deltaPose, Task* t, Task * controlGoal){
	math::applyAffineTrans(deltaPose, g);
	math::applyAffineTrans(-deltaPose, controlGoal);
	debug::print_pose(deltaPose, "delta pose");
}

int Configurator::motor_step(Task::Action a, float distance){
	int result=0;
        if (a.getOmega()>0){ //LEFT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
        }
		else if (a.getOmega()<0){ //RIGHT
            result = (SAFE_ANGLE)/(MOTOR_CALLBACK * a.getOmega());
		}
		else if (a.getLinearSpeed()>0){
			result = (distance)/(MOTOR_CALLBACK*a.getLinearSpeed());
		}
	    return abs(result);
    }


Task Configurator::task_to_execute(const std::vector<vertexDescriptor>&p, const TransitionSystem& g,  int end_it){
	Task t=controlGoal;
	if (p.empty()){
		return t;
	}	
	b2Transform start_to_end= g[p[0]].start - g[p[end_it]].endPose;
	if (Disturbance Dn= g[p[0]].Dn; Dn.getAffIndex()==AVOID && g[p[0]].direction==DEFAULT){
		//Disturbance Di= Dn;
		Dn.set_affordance(PURSUE);
		t=Task(Dn, g[p[0]].direction, b2Transform_zero, true);
		float distance = g[p[end_it]].end_from_Dn().p.Length();
		t.setEndCriteria(Distance(distance)); //set task to get within a certain distance from an object (as planned) and then terminate
	}
	else{
		t=Task(g[p[0]].Di, g[p[0]].direction, b2Transform_zero, true);

	}
	t.motorStep=motor_step(t.getAction(), start_to_end.p.Length());
	return t;

}



int Configurator::to_task_end(){
	int i=0;
	Direction d=transitionSystem[plan[i]].direction;
	while(i<plan.size() &&transitionSystem[plan[i]].direction==d){
		i++;
	}
	return i-1;
	
}
