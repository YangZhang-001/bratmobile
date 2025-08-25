#include "custom_robot.h"

#undef PLANNING
#define PLANNING false

class AffordanceSetter{
    AffordanceIndex affordance=NONE;
    public:
    AffordanceSetter()=default;
    AffordanceSetter(const char * text){
        init(text);
    }
    
    void init(const char * text){
	switch (text){
		case "AVOID":
			return AVOID;
		case "PURSUE":
			return PURSUE;
		case "NONE":
			return NONE;
        default:
            std::cout<<"WHAT?? VALID AFFORDANCE PLEASE"<<std::endl;
            std::cin >>text;
            return init(text);
            break;
	    }
    }

    AffordanceIndex getAffIndex(){return affordance;}
};

class DirectionSetter{
    protected:
    Direction direction=DEFAULT;
    public:
    DirectionSetter()=default;
    DirectionSetter(const char * text){
        init(text);
    }
    void init(const * char text){
	switch (text){
		case "LEFT":
			direction= LEFT;break;
		case "RIGHT":
			direction= RIGHT;break;
		case "DEFAULT":
			direction= DEFAULT;break;
	    }
        default:
            std::cout<<"WHAT?? VALID DIRECTION PLEASE"<<std::endl;
            std::cin >>text;
            return DirectionSetter(text);
            break;
    }

    Direction getDirection(){
        return direction;
    }
};



class UserInputConfigurator: public ReactiveConfigurator{
    protected:
    DirectionSetter *directionSetter=NULL;
    AffordanceSetter *affordanceSetter=NULL;


    void explore_plan(b2World &world){
        if (worldBuilder.get_world_objects().size()==0){
            std::cout<<"ADD AN OBSTACLE PLEASE!"<<std::endl;
            return;
        }
        if (iteration>1){
            ReactiveConfigurator::explore_plan(world);
        }
        else{
            Disturbance disturbance;
            if(affordanceSetter->getAffIndex()!=NONE){
                disturbance.bf=worldBuilder.get_world_objects()[0];
                disturbance.set_affordance(affordanceSetter->getAffIndex());
                disturbance.validate();
                currentTask=Task(disturbance, directionSetter->getDirection(), b2Transform_zero, true);
            }
        }
    }
    public:
    UserInputConfigurator()=delete;

    UserInputConfigurator(DirectionSetter * ds, AffordanceSetter * as): ReactiveConfigurator(){
        directionSetter=ds;
        affordanceSetter=as;
    }

    ~UserInputConfigurator(){
        directionSetter=NULL;
        affordanceSetter=NULL;
    }
};

class OneTaskController: public Reactive_Controller{
    protected:
    public:
    OneTaskController()=default;

    void next_task(Task & currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan)override{
        if (currentTask.is_over() && currentTask.getAction().getLWheelSpeed()!=0 && currentTask.getAction().getRWheelSpeed()!=0){
            currentTask=Task(Disturbance(), STOP);
            currentTask.getAction().setLWheelSpeed(0);
            currentTask.getAction().setRWheelSpeed(0);
            currentTask.set_change(false);
        }
    }
};


// Disturbance set_target(int& run, b2Transform start){
// 	Disturbance result;
// 	return result;
// }
#undef DEBUG
#define DEBUG true

int main(int argc, char** argv) {
	printf("PLANNING =%i\n", PLANNING);
	A1Lidar lidar;
	AlphaBot motors;
    Task controlGoal;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
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
	
	
