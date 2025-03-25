#include "custom.h"

#define PLANNING false
void forget(Configurator *c){}

class AffordanceSetter{
	public:
	AffordanceIndex affordance=NONE;
	AffordanceSetter()=default;
	AffordanceSetter(AffordanceIndex i):affordance(i){}

}as;

void Configurator::explore_plan(b2World &world){
	if (transitionSystem.m_vertices.size()==1 && iteration<=1){
		movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
		transitionSystem[movingVertex].direction=DEFAULT;
		currentTask.action.init(transitionSystem[currentVertex].direction);
	}
	if (currentTask.action.getOmega()!=0 && currentTask.motorStep<(transitionSystem[movingEdge].step)){
		return;
	}
	//adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
	worldBuilder.buildWorld(world, data2fp, transitionSystem[movingVertex].start, currentTask.direction); //was g[v].endPose
	simResult result = simulate(currentTask, world); //transitionSystem[currentVertex],transitionSystem[currentVertex],
	gt::fill(result, transitionSystem[currentVertex].ID, &transitionSystem[currentEdge]);
	transitionSystem[currentVertex].Dn.set_affordance(as.affordance);
	currentTask.change = transitionSystem[currentVertex].outcome!=simResult::successful;
	if (currentTask.change){
		printf("crashed\n");
	}

}

Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	return result;
}


class ReducedCallback :public AlphaBot::StepCallback { //every 100ms the callback updates the plan
    float L=0;
	float R=0;
public:
Configurator * c;


ReducedCallback(Configurator *conf): c(conf){
}
void step( AlphaBot &motors){
	if (c->getIteration() <=0){
		return;
	}
	if (!c->running){
		motors.setRightWheelSpeed(0);
 	    motors.setLeftWheelSpeed(0);		
	}
	ctr_mutex.lock();
	c->control->track_task_execution(*c->getTask(), c->transitionSystem, &(c->controlGoal), c->currentVertex, c->data2fp);
	printf("disturbance:");
	debug::print_pose(c->getTask()->disturbance.pose());
	EndedResult er = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, false);
	c->control->change_task(c->getTask()->change,  c->control->plan,c->transitionSystem, c->controlGoal, *c->getTask(), c->currentVertex);
	R= c->getTask()->getAction().getRWheelSpeed();
	L=c->getTask()->getAction().getLWheelSpeed(); //*1.05
	if (c->getTask()->direction==LEFT){
		R*=1.37; //23
		L*=1.37;
	}
	else if (c->getTask()->direction==RIGHT){
		R*=1.07; //17
		L*=1.07;
	}
	else if (c->getTask()->direction==DEFAULT){
		R*=1.15*1.1;
		L*=1.15;
	}
	ctr_mutex.unlock();	
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	printf(",R=%f\tL=%f\n",c->getTask()->getAction().getRWheelSpeed(), c->getTask()->getAction().getLWheelSpeed());
}
};


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	LIDAR_In configuratorInterface;
	Motor_IO controlInterface;
    Configurator configurator(controlGoal);
	if (argc>2){
		configurator.debugOn= atoi(argv[2]);
		configuratorInterface.debugOn = configurator.debugOn;
	}
	configurator.setSimulationStep(.5);
	as=AffordanceSetter(AffordanceIndex(atoi(argv[1])));
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	ReducedCallback cb(&configurator);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
