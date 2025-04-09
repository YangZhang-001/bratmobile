
#include "custom.h"


void forget(Configurator *c){}

Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	if (run%2==0){
		result=Disturbance(PURSUE, start.p, start.q.GetAngle());
		run++;
	}
	return result;
}
void Configurator::explore_plan(b2World&world){
    pre_explore(transitionSystem, plan, currentTask.change);
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi(transitionSystem);
    plan=explorer(src, transitionSystem, world);
    if (debugOn){
        std::vector<vertexDescriptor> _plan=(plan);
        debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, _plan, currentVertex);
    }		
    ts_cleanup(transitionSystem, plan); //remove self-edge and singleton states
    if (plan.empty() && (!transitionSystem[currentVertex].visited() || currentTask.change)){ //currentv not visited means that it wasn't observed ()
        printf("no plan, searchign from %i\n", src);
        bool finished=false;
        plan= planner(transitionSystem, currentVertex, TransitionSystem::null_vertex(), false, NULL, &finished); //src
    }
    else{
        printf("recycled plan in explorer:\n");
    }
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	LIDAR_In configuratorInterface;
	Motor_OUt controlInterface;
    Configurator configurator(controlGoal);
	char name[60];
	dump_benchmarks( "rt-update-targetless", "/tmp");
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}	configurator.setSimulationStep(.5);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start();
	motors.start();
	configurator.start();
	do {
	} while (!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();

}
	
	
