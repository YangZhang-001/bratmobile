#include "custom_robot.h"

#undef PLANNING
#define PLANNING false

class AffordanceSetter{
    AffordanceIndex affordance=NONE;
    public:
    AffordanceSetter(){
        std::cout<<"ENTER AFFORDANCE, CAPITALISED:"<<std::endl;
        std::string str;
        std::cin >>str;
        init(str);
    }

    AffordanceSetter(std::string str){
        init(str);
    }
    
    void init(std::string text){
	if (text== "AVOID") affordance= AVOID;
	else if (text=="PURSUE") affordance= PURSUE;
	else if (text== "NONE") affordance= NONE;
    else{
        std::cout<<"WHAT?? VALID AFFORDANCE PLEASE"<<std::endl;
        std::cin >>text;
        return init(text); 
        }    
    }

    AffordanceIndex getAffIndex(){return affordance;}
};

class DirectionSetter{
    protected:
    Direction direction=DEFAULT;
    public:
    DirectionSetter(){
        std::cout<<"ENTER DIRECTION, CAPITALISED:"<<std::endl;
        std::string str;
        std::cin >>str;
        std::cin.ignore();   // eat the leftover '\n'
        std::cin.get();
        init(str);
    }

    DirectionSetter(std::string str){
        init(str);
    }
    void init(std::string text){
	if (text== "LEFT") direction= LEFT;
	else if (text=="RIGHT") direction= RIGHT;
	else if (text== "DEFAULT") direction= DEFAULT;
	else{
        std::cout<<"WHAT?? VALID DIRECTION PLEASE"<<std::endl;
        std::cin >>text;
        std::cin.ignore();   // eat the leftover '\n'
        return init(text);
    }
    }

    Direction getDirection(){
        return direction;
    }
};



class UserInputConfigurator: public virtual Configurator{
    protected:
    DirectionSetter *directionSetter=NULL;
    AffordanceSetter *affordanceSetter=NULL;

    // void init(Task _task)override{
    //     if (!directionSetter){
    //         throw "no direction setter!";
    //     }
    //     if (!affordanceSetter){
    //         throw "no affordance setter";
    //     }
    //     if (directionSetter->getDirection()==DEFAULT && affordanceSetter->getAffIndex()==AVOID){
            
    //     }
    // }


    void explore_plan(b2World &world)override{
        if (worldBuilder.get_world_objects().size()==0){
            std::cout<<"ADD AN OBSTACLE PLEASE!"<<std::endl;
            return;
        }
        if (worldBuilder.get_world_objects().size()>1){
            throw "TOO MANY OBSTACLES!!";
        }
        std::cout<<iteration<<std::endl;
        if (iteration<=1){
            simResult result;            
            Disturbance disturbance;
            disturbance.bf=worldBuilder.get_world_objects()[0];
            disturbance.set_affordance(affordanceSetter->getAffIndex());
            disturbance.validate();
            vertexDescriptor v1=boost::add_vertex(transitionSystem);
            auto e=boost::add_edge(currentVertex, v1, transitionSystem);
            transitionSystem[v1].direction=directionSetter->getDirection();
            if(affordanceSetter->getAffIndex()==PURSUE && transitionSystem[v1].direction==DEFAULT){
                transitionSystem[v1].Dn=disturbance;
                transitionSystem[v1].Di=controlGoal.get_disturbance();
                transitionSystem[v1].endPose.p.x=disturbance.pose().p.x-0.07;
            }
            else{
                transitionSystem[v1].Di=disturbance;
                if (transitionSystem[v1].direction==DEFAULT){
                    float howFarShift=.5;
                    if (disturbance.pose().p.y<0) howFarShift=-howFarShift;
                    std::cout<<"howfar="<<howFarShift<<std::endl;
                    b2Transform newGoal;
                    newGoal.p=b2Vec2(howFarShift, 0)+disturbance.pose().p;
                    controlGoal=Task(Disturbance(PURSUE, newGoal.p), UNDEFINED);
                    init(controlGoal);
                    register_tracker(tracker);
                }
            }
            currentTask.set_change(true);
            transitionSystem[e.first].step=20;
            transitionSystem[e.first].it_observed=iteration;
            debug::print_pose(transitionSystem[v1].Di.pose(), "Di:");
            debug::print_pose(transitionSystem[v1].Dn.pose(), "Dn:");
            m_plan={v1};
        }
        
    }
    public:
    UserInputConfigurator()=delete;

    UserInputConfigurator(DirectionSetter * ds, AffordanceSetter * as): Configurator(){
        directionSetter=ds;
        affordanceSetter=as;
    }

    ~UserInputConfigurator(){
        directionSetter=NULL;
        affordanceSetter=NULL;
    }
};

class OneTaskController: public Wise_Controller{
    protected:
    public:
    OneTaskController()=default;

    void next_task(Task & currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan)override{
        Wise_Controller::next_task(currentTask, controlGoal, g, current_vertices, plan);
        // if (currentTask.is_over() && currentTask.getAction().getLWheelSpeed()!=0 && currentTask.getAction().getRWheelSpeed()!=0){
        //     std::cout<<"terminating!"<<std::endl;
        //     currentTask=Task(Disturbance(), STOP);
        //     currentTask.getAction().setLWheelSpeed(0);
        //     currentTask.getAction().setRWheelSpeed(0);
        //     currentTask.set_change(false);
        // }
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
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputConfigurator configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    Task controlGoal(goal, UNDEFINED);
    configurator.init(controlGoal);
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
	do{
    }while(!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
