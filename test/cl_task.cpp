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
int ogStep=0;
Motor_Out * mio;
int run=0;

ReducedCallback(Motor_Out *_mio): mio(_mio){
}
void step( AlphaBot &motors){
	// if (mio->iteration <=0){
	// 	return;
	// }
	if (!mio->running){
		motors.setRightWheelSpeed(0);
 	    motors.setLeftWheelSpeed(0);		
	}
	// mio->setReady(false);
	// mio->track_task_execution();
	// mio->change_task(mio->task.change,  mio->plan);
	// printf("changed\n");
	// R= mio->task.getAction().getRWheelSpeed();
	// L=mio->task.getAction().getLWheelSpeed(); //*1.05
	// mio->setReady(true);
    motors.setRightWheelSpeed(R); //temporary fix because motors on despacito are the wrong way around
    motors.setLeftWheelSpeed(L);
	printf(",R=%f\tL=%f\n",mio->task.getAction().getRWheelSpeed(), mio->task.getAction().getLWheelSpeed());
}
};


int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    Configurator configurator(controlGoal);
	if (argc>2){
		configurator.debugOn= atoi(argv[2]);
		configuratorInterface.debugOn = configurator.debugOn;
	}
	configurator.setSimulationStep(.5);
	as=AffordanceSetter(AffordanceIndex(atoi(argv[1])));
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	ReducedCallback cb(&controlInterface);
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
	
	
