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
	task.disturbance.bf.pose=b2Mul(B, task.disturbance.bf.pose);

}

void Configurator::init(Task _task){
	controlGoal=_task;
	currentTask=_task;
	register_tracker(tracker);
	transitionSystem[MOVING_VERTEX].Di=controlGoal.disturbance;
	currentVertex=MOVING_VERTEX;
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

Robot Configurator::makeRobot(b2World& world, const Task & task){
	Robot robot(&world);
	robot.body()->SetTransform(task.start.p, task.start.q.GetAngle());
	return robot;

};

simResult Configurator::simulate(Task  t, b2World & w){ //State& state, State src, 
	simResult result;
	float remaining=remainingSimulationTime(&t);
	Robot robot=makeRobot(w, t);
	worldBuilder->add_body_count();
	simulatedTasks++;
	result =t.bumping_that(w, iteration, robot.body(), remaining); //default start from 0
	//approximate angle to avoid rounding errors
	result.endPose.q.Set(approximate_angle(result.endPose.q.GetAngle(), t.direction, result.resultCode));
	if (iteration<1 && isCurrentTask(t) && calibrationCallback){
		calibrationCallback->simulationReady(result.collision);
	}
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


void Configurator::registerInterface(MotorInterface * _control){
	control=_control;
}

void Configurator::newScanEvent(){
		if (!areInterfacesSetUp()){
			return;
		}
		Spawner();
		if (getIteration()>1){
			TrackingResult trackingResult(currentTask.get_disturbance());
			trackingResult= tracker->track((currentTask),data2fp, worldBuilder->get_world_objects());
			update_graph(transitionSystem, trackingResult);
		}
		tracker->on_new_reading(currentTask, controlGoal);
		if (goal_changer!=NULL){
			if (( currentTask.is_over()& transitionSystem[currentVertex].direction!=STOP && m_plan.empty() && getIteration()>1)){
				controlGoal=goal_changer->change_goal(controlGoal);
			}					
		}
		change_task();		
		//adjust_goal_expectation(); //dubious if this is needed tbh
		estimate_current_vertex();
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
	currentTask.disturbance=tr.observed_disturbance;
	if (DEBUG){
		char goal[40];
		sprintf(goal, "/tmp/goal%04i.txt", iteration);
		FILE * goalFile=fopen(goal, "w");
		fprintf(goalFile, "%0.3f\t%0.3f\n", controlGoal.disturbance.getPosition().x, controlGoal.disturbance.getPosition().y);
		fclose(goalFile);
	}
		if (!tracker){
		std::cout <<"tracker uninitialised!";
		return;
	}
	if (tracker->hasTaskEnded(currentTask)){
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

bool Configurator::areInterfacesSetUp(){
	if (control == NULL){
		std::cerr<<"null pointer to motor output";
		return false;
	}
	if (task_controller==NULL){
		std::cerr<<"no task controller, please set!";
		return false;
	}
	if (!tracker){
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

bool Configurator::isCurrentTask(const Task & t){
	return t.direction==currentTask.direction && t.start==b2Transform_zero && t.disturbance.getAffIndex()==currentTask.getAffIndex();
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


