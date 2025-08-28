#ifndef REAL_TEST_H
#define REAL_TEST_H
#include "attentive.h"


class AffordanceSetter{
    AffordanceIndex affordance=NONE;
    public:
    AffordanceSetter(){
        std::cout<<"ENTER AFFORDANCE, CAPITALISED:"<<std::endl;
        std::string str;
        std::cin >>str;
        init(str);
    }

    AffordanceSetter(AffordanceIndex a): affordance(a){}

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

    DirectionSetter(Direction d): direction(d){}

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
            disturbance.validate();
            vertexDescriptor v1=boost::add_vertex(transitionSystem);
            auto e=boost::add_edge(currentVertex, v1, transitionSystem);
            transitionSystem[v1].direction=directionSetter->getDirection();
            if(affordanceSetter->getAffIndex()==PURSUE && transitionSystem[v1].direction==DEFAULT){
                disturbance.set_affordance(AVOID);
                transitionSystem[v1].Dn=disturbance;
                transitionSystem[v1].Di=controlGoal.get_disturbance();
                transitionSystem[v1].endPose.p.x=disturbance.pose().p.x-0.07;
            }
            else{
                disturbance.set_affordance(affordanceSetter->getAffIndex());
                transitionSystem[v1].Di=disturbance;
                if (transitionSystem[v1].direction==DEFAULT){
                    float howFarShift=.5;
                    if (disturbance.pose().p.y<0) howFarShift=-howFarShift;
                    std::cout<<"howfar="<<howFarShift<<std::endl;
                    b2Transform newGoal;
                    newGoal.p=b2Vec2(0, howFarShift)+disturbance.pose().p;
                    debug::print_pose(newGoal, "newgoal");
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
    public:
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

class DebugTracker: public ClosedLoop_Tracker{
    void printWindow(){
        b2AABB aabb;
        attention_window.ComputeAABB(&aabb, b2Transform_zero, 0);
        std::cout<<"upperbound="<<aabb.upperBound.x<<", "<<aabb.upperBound.y<<std::endl;
        std::cout<<"lowerbound="<<aabb.lowerBound.x<<", "<<aabb.lowerBound.y<<std::endl;

    }
public:
void on_new_reading(Task * goal){
    ClosedLoop_Tracker::on_new_reading(goal);
    printWindow();
}
};

#endif