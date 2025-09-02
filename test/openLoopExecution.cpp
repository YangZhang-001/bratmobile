#include "realWorldTestHeaders.h"
#include "../custom_robot.h"
/**
 * Tracks and executes tasks using deadreckoning
 */
class OpenLooper: public DeadReckoner, public Motor_Out, public MotorCallback{
    int motorStep=0;
    public:
    OpenLooper():Motor_Out(this){}

    void on_new_task(const Task &task, const Task & goal){
        motorStep=task.getMotorStep();
        deltaTransform=b2Transform_zero;
    }

    void step(Alphabot& motors){
        MotorCallback::step(motors);
        if (L!=0 && R!=0){
            motorStep--;
        }
    }
};

class UserInputDR:public UserInputConfigurator{
    void getTaskFromInput(){
        Disturbance disturbance;
        disturbance.bf=worldBuilder.get_world_objects()[0];
        disturbance.set_affordance(as->getAffIndex());
        disturbance.validate();
        Task task(disturbance, ds->getDirection(), b2Transform_zero, true);
        b2World world;
        worldBuilder.buildWorld(world, task.start, task.getDirection(), disturbance);
        if(ds->getDirection()==PURSUE && as->getAffIndex()){
            task.setEndCriteria(Distance(0.14));
        }
        simResult sr=simulate(task, world);
        vertexDescriptor v1=boost::add_vertex(transitionSystem);
        auto e=boost::add_edge(currentVertex, v1, transitionSystem);
        transitionSystem[v1].direction=directionSetter->getDirection();
        transitionSystem[e.first].step=sr.step;
        m_plan={v1};
        currentTask.set_change(true);
        transitionSystem[e.first].it_observed=iteration;

    }

};

class OpenLoopController:public Controller{
    Task next_task(const Task & currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan){
        if (plan.empty()){
        //printf("I DON'T KNOW WHAT TO DO NOW\n");
	        return stopTask(controlGoal);
        }   

        Task result(g[plan[0]].direction, Disturbance(), b2Trasform_zero, true);
        auto e=boost::edge(0, plan[0], g);
        currentTask.setMotorStep(g[e.first].step);
        return currentTask;
    }
}

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	LIDAR_In configuratorInterface;
	// Motor_Out controlInterface;
    OpenLooper openLooper;
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputConfigurator configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    // ClosedLoop_Tracker tracker;
    Task controlGoal(goal, UNDEFINED);
    configurator.register_tracker(&openLooper);
    configurator.init(controlGoal);
	OneTaskController rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	//printf("current vertices size=%i\n", configurator.current_vertices.size());

	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &openLooper);
	MotorCallback cb(&openLooper);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	do{
    }while(!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
