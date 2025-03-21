#include "custom.h"

void forget(Configurator *c){}

class AffordanceSetter{
	public:
	AffordanceIndex affordance=NONE;
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
	transistionSystem[currentVertex].Dn.set_affordance(as.affordance);
	currentTask.change = transitionSystem[currentVertex].outcome!=simResult::successful;
	if (currentTask.change){
		printf("crashed\n");
	}

}

Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	return result;
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal();
	ConfiguratorInterface configuratorInterface;
	ControlInterface controlInterface;
    Configurator configurator(controlGoal);
	configurator.planning =0;
	if (argc>2){
		configurator.debugOn= atoi(argv[2]);
		configuratorInterface.debugOn = configurator.debugOn;
	}
	configurator.setSimulationStep(.5);
	as=AffordanceSetter(AffordanceIndex(atoi(argv[1])))
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&configurator);
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
	
	
