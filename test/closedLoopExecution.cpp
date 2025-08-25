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
	if (text== "AVOID") affordance= AVOID;
	else if (text=="PURSUE") affordance= PURSUE;
	else if (text== "NONE") affordance= NONE;
    else{
        std::cout<<"WHAT?? VALID AFFORDANCE PLEASE"<<std::endl;
        std::string str;
        std::cin >>str;
        return init(str.c_str()); 
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
    void init(const char* text){
	if (text== "LEFT") direction= LEFT;
	else if (text=="RIGHT") direction= RIGHT;
	else if (text== "DEFAULT") direction= DEFAULT;
	else{
        std::cout<<"WHAT?? VALID DIRECTION PLEASE"<<std::endl;
        std::string str;
        std::cin >>str;
        return init(str.c_str());
    }
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
        if (currentTask.getAffIndex()!=NONE){
            ReactiveConfigurator::explore_plan(world);
        }
        else{
            Disturbance disturbance;
            if(affordanceSetter->getAffIndex()!=NONE){
                disturbance.bf=worldBuilder.get_world_objects()[0];
                disturbance.set_affordance(affordanceSetter->getAffIndex());
                disturbance.validate();
                currentTask=Task(disturbance, directionSetter->getDirection(), b2Transform_zero, true);
               if (directionSetter->getDirection()==DEFAULT){
                    if (affordanceSetter->getAffIndex()==AVOID){
                    currentTask=Task(Disturbance(PURSUE, b2Vec2(1.0, 0), UNDEFINED));
                    }
                    else if (affordanceSetter->getAffIndex()==PURSUE){
                        currentTask.setEndCriteria(Distance(0.07)); //go 7cm close to the obstacle
                    }
                }
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
	A1Lidar lidar;
	AlphaBot motors;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    AffordanceSetter as(argv[1]);
    DirectionSetter ds(argv[1]);
    UserInputConfigurator configurator(&ds, &as);
	ClosedLoop_Tracker tracker;
	configurator.register_tracker(&tracker);
	OneTaskController rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	//printf("current vertices size=%i\n", configurator.current_vertices.size());

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
	
	
