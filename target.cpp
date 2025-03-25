#include "custom.h"

void forget(Configurator *c){}

void Configurator::explore_plan(b2World&world){
    pre_explore(transitionSystem, control->plan, currentTask.change);
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi(transitionSystem);
    control->plan=explorer(src, transitionSystem, world);
    if (debugOn){
        std::vector<vertexDescriptor> _plan=(control->plan);
        debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, _plan, currentVertex);
    }		
    ts_cleanup(transitionSystem, control->plan); //remove self-edge and singleton states
    if (control->plan.empty() && (!transitionSystem[currentVertex].visited() || currentTask.change)){ //currentv not visited means that it wasn't observed ()
        printf("no plan, searchign from %i\n", src);
        bool finished=false;
        control->plan= planner(transitionSystem, currentVertex, TransitionSystem::null_vertex(), false, NULL, &finished); //src
    }
    else{
        printf("recycled plan in explorer:\n");
    }
}


Disturbance set_target(int& run, b2Transform start){
	Disturbance result;
	if (run%2!=0){
		result= Disturbance(PURSUE, b2Vec2(1.0f, 0.0f), 0.0f);
	}
	else{
		result= Disturbance(PURSUE, b2Vec2(-1.0f, 0.0f), M_PI);
	}
	return result;
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
	Motor_IO controlInterface;
    Configurator configurator(controlGoal);
	configurator.setBenchmarking(1, "rt-update", "/tmp");
	if (argc>1){
		configurator.debugOn= atoi(argv[1]);
		configuratorInterface.debugOn = atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
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
	
	
