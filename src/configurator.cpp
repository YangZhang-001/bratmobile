#include "configurator.h"
#include <chrono>

void Configurator::MulT(const b2Transform& B, Task& task){
	math::MulT(B, task.start);
	math::MulT(B, task.disturbance);
}

void Configurator::InvMul(const b2Transform& B, Task& task){
	math::InvMul(B, task.start);
	math::InvMul(B, task.disturbance);
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
	current_vertices={currentVertex};
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

bool Configurator::Spawner(){ 
	iteration++; //iteration set in getVelocity
	worldBuilder->add_iteration();
	simulatedTasks=0;
	//BENCHMARK + FIND TRUE SAMPLING RATE
	auto now =std::chrono::high_resolution_clock::now();

	//CREATE BOX2D ENVIRONMENT
	b2World world= b2World(GRAVITY);
	char name[256];
	worldBuilder->set_world_objects(worldBuilder->getFeatures(data2fp, b2Transform_zero, WorldBuilder::PARTITION));
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d_getFeatures= now- endTime; //in seconds
	float duration_getFeatures=abs(float(d_getFeatures.count())/1000); //express in seconds
	explore_plan(world);
	endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d_withExplore= now- endTime; //in seconds
	float duration_withExplore=abs(float(d_withExplore.count())/1000); //express in seconds
	//FORMAT: vertices	bodies tasks	total_dur	just_worldbuilding
	if (logger){
		logger->log("%i\t%i\t%i\t%0.6f\t%0.6f\n", transitionSystem.m_vertices.size(), worldBuilder->bodies, simulatedTasks, duration_withExplore, duration_getFeatures);
	}
	worldBuilder->resetBodies();
	return 1;
}

Robot Configurator::makeRobot(b2World& world, const b2Transform & start){
	Robot robot(&world);
	robot.body()->SetTransform(start.p, start.q.GetAngle());
	return robot;

};

simResult Configurator::simulate(Task  t, b2World & w){ //State& state, State src, 
	simResult result;
	float remaining=remainingSimulationTime(&t);
	printf("remaining=%f\n", remaining);
	Robot robot=makeRobot(w, t.start);
	worldBuilder->add_body_count();
	simulatedTasks++;
	result =t.bumping_that(w, iteration, robot.body(), remaining); //default start from 0
	//approximate angle to avoid rounding errors
	//b2Transform travelTransform=b2MulT(result.endPose, t.start);
	result.endPose.q.Set(approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode));
	return result;
}

float Configurator::remainingSimulationTime(const Task *const t){
	float distance=BOX2DRANGE;
	if (controlGoal.disturbance.isValid()){
		distance= controlGoal.disturbance.getPosition().Length();
	}
	return distance/controlGoal.action.getLinearSpeed();

}



std::pair<edgeDescriptor, bool> Configurator::addVertex(const vertexDescriptor & src, vertexDescriptor &v1, Edge edge, bool topDown){ //returns edge added
	std::pair<edgeDescriptor, bool> result;
	result.second=false;
	if (transitionSystem[src].options.size()>0 || topDown){
		v1 = boost::add_vertex(transitionSystem);
		result = boost::add_edge(src, v1, transitionSystem);
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
		if (!c->areInterfacesSetUp(c)){
			c->running=false;
		}
		if (c->ci->stop){
			c->ci=NULL;
			c->control=NULL;
			printf("ci not started\n");
			c->running=false;
		}		
		if (c->ci->isReady()){
			c->ci->setReady(false);
			c->data2fp= CoordinateContainer(c->ci->data2fp);
			c->Spawner();
			if (c->getIteration()>1){
				TrackingResult trackingResult(c->currentTask.get_disturbance());
				trackingResult= c->tracker->track((c->currentTask),c->ci->data2fp, c->worldBuilder->get_world_objects());
				c->update_graph(c->transitionSystem, trackingResult);
			}
			if (c->goal_changer!=NULL){
				if (( c->currentTask.is_over()& c->transitionSystem[c->currentVertex].direction!=STOP && c->m_plan.empty() && c->getIteration()>1)){
					c->controlGoal=c->goal_changer->change_goal(c->controlGoal);
				}					
			}
			c->change_task();		
			c->adjust_goal_expectation();
			c->estimate_current_vertex();
			printf("current v=%i\n", c->currentVertex);
			c->tracker->on_new_reading(c->controlGoal, c->currentTask);
			}

	}

}
float Configurator::approximate_angle(float angle, Direction d, simResult::resultType outcome){
	float result=angle, decimal=0, integer=0;
	if ((d==LEFT || d==RIGHT)&& outcome!=simResult::crashed){
		float ratio= angle/(ANGLE_RESOLUTION);
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


void Configurator::estimate_current_vertex(){
	if(current_vertices.empty() ){
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
	currentTask=task_controller->next_task(currentTask, controlGoal, transitionSystem, current_vertices, m_plan);
	//transitionSystem[movingEdge].step=currentTask.getMotorStep();
	std::cout<<"new task step= "<<currentTask.getMotorStep()<<std::endl;
	tracker->on_new_task(currentTask, controlGoal);
	if (control){
		control->reset();
		control->getData(currentTask.action);
	}
	else{
		std::cerr<<("no motor interface found");
	}
	return;
}

void Configurator::update_graph(TransitionSystem&g, const TrackingResult & tr){
	math::InvMul(tr.displacement, g);
	Configurator::InvMul(tr.displacement, controlGoal);
	debug::print_pose(controlGoal.disturbance.pose(), "goal disturbance after tracking:");
	currentTask.disturbance=tr.observed_disturbance;
	if (!tracker){
		std::cout <<"tracker uninitialised!";
		return;
	}
	if (tracker->hasTaskEnded(currentTask, transitionSystem[currentVertex])){
		currentTask.change=true;
	}
}


void Configurator::adjust_goal_expectation(){
	if (controlGoal.getAffIndex()==PURSUE && !m_plan.empty()&&task_controller->get_disturbance().getAffIndex()!=NONE){
		b2Transform from_Di=b2Transform_zero;
		from_Di=currentTask.from_Di();
		b2Transform goal_robotPOV= b2Mul(from_Di,task_controller->disturbance_to_goal()); //position of goal from the robot based on where it should be from Di
		controlGoal.disturbance.bf.pose=goal_robotPOV;
		debug::print_pose(controlGoal.disturbance.pose(), "goal after adjusting");
		printf("distance after adjusting %f\n", controlGoal.disturbance.pose().p.Length());
	}

}

bool Configurator::areInterfacesSetUp(Configurator * c){
	if (c == NULL){
		std::cerr<<"null pointer to configurator";
		return false;
	}	
	if (c->ci == NULL){
		std::cerr<<"null pointer to lidar input";
		return false;
	}
	if (c->control == NULL){
		std::cerr<<"null pointer to motor output";
		return false;
	}
	if (c->task_controller==NULL){
		std::cerr<<"no task controller, please set!";
		return false;
	}
	if (!c->tracker){
		std::cerr<<"no tracker!";
		return false;
	}
	return true;

}

void Configurator::assignBodyFeatures(Task & t, const BodyFeatures & bf){
	t.disturbance.bf=bf;
}

void Configurator::assignDimensions(Task & t, float halfLength, float halfWidth){
	BodyFeatures bf=t.get_disturbance().bf;
	bf.halfLength=halfLength;
	bf.halfWidth=halfWidth;
	assignBodyFeatures(t, bf);
}

void Configurator::adjust_simulated_task(const vertexDescriptor &v, Task & t){
	std::pair<edgeDescriptor, bool> ep= boost::edge(v, currentVertex, transitionSystem);
	if(!ep.second){ //no tgt	
		return; //check until needs to be checked
	}
	if (!t.getEndCriteria().angle.isValid()){return;}
	if (t.get_direction()==DEFAULT){
		return;}
	if (t.get_direction()==currentTask.get_direction()){
		t.getEndCriteria().adjust(tracker->getDeltaTransform());
	}
	else if (t.get_direction()==getOppositeDirection(currentTask.get_direction()).second){
		t.getEndCriteria().adjust(-tracker->getDeltaTransform());
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

void ReactiveConfigurator::explore_plan(b2World &world){
	if (iteration<=1){
		movingEdge = boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem).first;
		Direction def=DEFAULT;
		transitionSystem[MOVING_VERTEX].direction=DEFAULT;
		currentTask.getAction().init(def);
	}
	Task t(currentTask.get_disturbance(), currentTask.get_direction(), b2Transform_zero, true);
	worldBuilder->buildWorld(world, b2Transform_zero, currentTask.get_direction()); //was g[v].endPose
	adjust_simulated_task(currentVertex, t);
	simResult result = simulate(t, world); //transitionSystem[currentVertex],transitionSystem[currentVertex],
	printf("crashed=%i, step=%i\n", result.resultCode==simResult::crashed, result.step);
	gt::fill(result, &transitionSystem[currentVertex], &transitionSystem[currentEdge]);
	currentTask.set_change(transitionSystem[currentVertex].outcome!=simResult::successful);
}

float ReactiveConfigurator::remainingSimulationTime(const Task *const t){
    if (!t){
		throw "no task!";
	}
	if (get_direction(t)==DEFAULT){
    	float r_step=Controller::motor_step(t->getAction(),simulationStep)*MOTOR_CALLBACK;
	 	printf("r_step=%i\n", r_step);
		return 	r_step;
    }
    return Configurator::remainingSimulationTime();
}


