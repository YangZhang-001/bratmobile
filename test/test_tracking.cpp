#include "custom_robot.h"


// void Configurator::explore_plan(b2World&world){
// 	debug::print_pose(controlGoal.disturbance.pose(), "Goal at:");
// 	printf("distance from goal=%f\n", controlGoal.disturbance.pose().p.Length());
// 	if (iteration>1){
// 		return;
// 	}
// 	auto startTime =std::chrono::high_resolution_clock::now();
//     pre_explore(transitionSystem, plan, currentTask.change);
//     vertexDescriptor src=get_explore_start(transitionSystem);
//     resetPhi(transitionSystem);
//     plan=explorer(src, transitionSystem, world);
//     if (DEBUG){
//         std::vector<vertexDescriptor> _plan=(plan);
//         debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, _plan, currentVertex);
//     }		
//     ts_cleanup(transitionSystem, plan); //remove self-edge and singleton states
//     if (plan.empty() && (!transitionSystem[currentVertex].visited() || currentTask.change)){ //currentv not visited means that it wasn't observed ()
//         printf("no plan, searchign from %i\n", src);
//         bool finished=false;
//         plan= planner(transitionSystem, currentVertex, TransitionSystem::null_vertex(), false, NULL, &finished); //src
//     }
//     else{
//         printf("recycled plan in explorer:\n");
//     }
// 	float duration=0; //tine for planning and exploring
// 	bool explored=0;
// 	auto endTime =std::chrono::high_resolution_clock::now();
// 	std::chrono::duration<float, std::milli>d= startTime- endTime; //in seconds
// 	duration=abs(float(d.count())/1000); //express in seconds
// 	printPlan(&plan);
// 	if (BENCHMARKING){
// 		FILE * f = fopen(statFile, "a+");
// 		if (explored){
// 			debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, plan, currentVertex);
// 			fprintf(f, "*");
// 		}
// 		fprintf(f,"%i\t%i\t%f\n", worldBuilder.getBodies(), transitionSystem.m_vertices.size(), duration);
// 		fclose(f);
// 	}
// 	printf("bodies = %i\n", worldBuilder.getBodies());
// }


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


/**
 * @brief This learner does not learn anything but only updates the threshold by expanding to accommodate error
 * 
 */
class LearningNothing:public ThresholdLearner{
	void Di_tune(const Bundle & error, const Bundle & x){
    // Di_weights=Di_weights+x*mu*error;
	// log();
	}

	Bundle update_bundle(const Bundle & error, const Bundle & x){
		return x+linear_rectify(error);
	}
};

int main(int argc, char** argv) {
	C1Lidar lidar;
	
	Disturbance target(2, b2Vec2(BOX2DRANGE, 0));
    Task controlGoal(target, DEFAULT);
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
	LearningNothing learner;
    AttentiveConfigurator configurator(controlGoal);
	ClosedLoop_Tracker tracker;
	configurator.register_tracker(&tracker);
	Wise_Controller wc;
	configurator.register_controller(&wc);
	tracker.register_learner(&learner);
	dump_benchmarks( "rt-update", "/tmp");
	if (argc>1){
		configuratorInterface.debugOn=atoi(argv[1]);
	}
	configurator.setSimulationStep(.27);
	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	tracker.make_log();
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	printf("all registered\n");
	configurator.start();
	lidar.start(C1Lidar::RPI_SERIAL_DEV);
	tracker.start();
	getchar();
	tracker.stop();
	configurator.stop();
	lidar.stop();
}
	
	
