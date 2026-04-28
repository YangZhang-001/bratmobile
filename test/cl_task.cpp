#include "custom_robot.h"

#undef PLANNING
#define PLANNING false


class AffordanceSetter{
	public:
	AffordanceIndex affordance=NONE;
	AffordanceSetter()=default;
	AffordanceSetter(AffordanceIndex i):affordance(i){}

}as;

class TaskSetter{
	public:
	Direction d=DEFAULT;
	bool topDown=false;

	TaskSetter()=default;

	TaskSetter(Direction _d):d(_d){
		topDown=true;
	}
	
}ts;

void Configurator::explore_plan(b2World &world){
       if (PLANNING){
               throw std::invalid_argument("wtf");
       }
       if (transitionSystem.m_vertices.size()==1 && iteration<=1){
               movingEdge = boost::add_edge(movingVertex, currentVertex, transitionSystem).first;
               transitionSystem[movingVertex].direction=DEFAULT;
               currentTask.action.init(transitionSystem[currentVertex].direction);
       }
       if (currentTask.action.getOmega()!=0 && currentTask.motorStep<(transitionSystem[movingEdge].step)){
               return;
       }
       //adjustStepDistance(currentVertex, transitionSystem, &currentTask, _simulationStep);
       worldBuilder.buildWorld(world, transitionSystem[movingVertex].start, currentTask.direction); //was g[v].endPose
       Task t=currentTask;
       t.H(t.disturbance, t.direction, true);
       simResult result = simulate(t, world); //transitionSystem[currentVertex],transitionSystem[currentVertex],
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
#define DEBUG true

int main(int argc, char** argv) {
	#undef PLANNING
	#define PLANNING false
	printf("PLANNING =%i\n", PLANNING);
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	MotorInterface controlInterface;
    Configurator configurator(controlGoal);
	ClosedLoop_Tracker tracker;
	configurator.register_tracker(&tracker);
	Reactive_Controller rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	//printf("current vertices size=%i\n", configurator.current_vertices.size());
	as=AffordanceSetter(AffordanceIndex(atoi(argv[1])));
	if (argc>3){
		ts =TaskSetter(Direction(atoi(argv[3])));
	}
	LidarInterface dataInterface(&configurator);
	configurator.registerInterface( &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	lidar.start(rpi_serial_dev);
	motors.start();
	getchar();
	motors.stop();
	lidar.stop();
}
	
	
