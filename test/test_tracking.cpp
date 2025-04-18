#include "custom.h"


void Configurator::explore_plan(b2World&world){
	if (iteration>1){
		return;
	}
	auto startTime =std::chrono::high_resolution_clock::now();
    pre_explore(transitionSystem, plan, currentTask.change);
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi(transitionSystem);
    plan=explorer(src, transitionSystem, world);
    if (DEBUG){
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
	float duration=0; //tine for planning and exploring
	bool explored=0;
	auto endTime =std::chrono::high_resolution_clock::now();
	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
	duration=abs(float(d.count())/1000); //express in seconds
	printPlan(&plan);
	if (BENCHMARKING){
		FILE * f = fopen(statFile, "a+");
		if (explored){
			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, plan, currentVertex);
			fprintf(f, "*");
		}
		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), transitionSystem.m_vertices.size(), duration);
		fclose(f);
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

void Configurator::next_task(){
	follow_plan();
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    Configurator configurator(controlGoal);
	dump_benchmarks( "rt-update", "/tmp");
	if (argc>1){
		#define DEBUG atoi(argv[1])
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	printf("all registered\n");
	configurator.start();
	lidar.start();
	motors.start();
	getchar();
	motors.stop();
	configurator.stop();
	lidar.stop();
}
	
	
