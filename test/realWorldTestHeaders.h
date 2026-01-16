#ifndef REAL_TEST_H
#define REAL_TEST_H
#include "unit_test/test_classes.h"


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



class UserInputConfigurator: public virtual DebugConfigurator{
    protected:
    DirectionSetter *directionSetter=NULL;
    AffordanceSetter *affordanceSetter=NULL;

    void explore_plan(b2World &world)override{
        if (worldBuilder->get_world_objects().size()==0){
            std::cout<<"ADD AN OBSTACLE PLEASE!"<<std::endl;
            return;
        }
        if (worldBuilder->get_world_objects().size()>1){
            std::cout<<"TOO MANY OBSTACLES!!"<<std::endl;
        }
        if (iteration<=1){
            getTaskFromInput();
        }
        
    }

    virtual void getTaskFromInput(){
        Disturbance disturbance;
        disturbance.bf=worldBuilder->get_world_objects()[0];
        disturbance.validate();
        vertexDescriptor v1=boost::add_vertex(transitionSystem);
        auto e=boost::add_edge(currentVertex, v1, transitionSystem);
        transitionSystem[v1].direction=directionSetter->getDirection();
        if(affordanceSetter->getAffIndex()==PURSUE && transitionSystem[v1].direction==DEFAULT){
            //set distance for how close to come to an obstacle
            disturbance.set_affordance(AVOID);
            transitionSystem[v1].Dn=disturbance;
            transitionSystem[v1].Di=controlGoal.get_disturbance();
            transitionSystem[v1].endPose.p.x=disturbance.pose().p.x-0.07;
        }
        else{
            disturbance.set_affordance(affordanceSetter->getAffIndex());
            transitionSystem[v1].Di=disturbance;
            if (transitionSystem[v1].direction==DEFAULT){
                assignNewGoal(disturbance);
                transitionSystem[v1].endPose.p.x=.2; //let's say it moved 20 cm
                Task task(disturbance, directionSetter->getDirection(), b2Transform_zero, true);
                b2World world(GRAVITY);
                simResult sr=simulate(task, world);
                std::cout<<"steps from simulation:"<<sr.step<<std::endl;
            }
            if (affordanceSetter->getAffIndex()==PURSUE){
                //set pose for turns in pursuit of a disturbance
                if (transitionSystem[v1].direction==LEFT ){
                    transitionSystem[v1].endPose.q.Set(M_PI_2);
                }
                else if (transitionSystem[v1].direction==RIGHT){
                    transitionSystem[v1].endPose.q.Set(-M_PI_2);
                }
            }
        }
        currentTask.set_change(true);
        transitionSystem[e.first].step=1;
        transitionSystem[e.first].it_observed=iteration;
        debug::print_pose(transitionSystem[v1].Di.pose(), "Di:");
        debug::print_pose(transitionSystem[v1].Dn.pose(), "Dn:");
        m_plan={v1};

    }

    void assignNewGoal(Disturbance &disturbance){
        float howFarShift=.5;
        if (disturbance.pose().p.y<0) howFarShift=-howFarShift;
        b2Transform newGoal; //goal in line with the obstacle
        newGoal.p=b2Vec2(0, howFarShift)+disturbance.pose().p;
        controlGoal=Task(Disturbance(PURSUE, newGoal.p), UNDEFINED); //set goal
        init(controlGoal);
        register_tracker(tracker); //make tracker to track the new goal

    }


    public:

    UserInputConfigurator(){}
    UserInputConfigurator(DirectionSetter * ds, AffordanceSetter * as): DebugConfigurator(){
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

    Task next_task(Task currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan)override{
        currentTask= Wise_Controller::next_task(currentTask, controlGoal, g, current_vertices, plan);
        if (currentTask.is_over() && currentTask.getAction().getLWheelSpeed()!=0 && currentTask.getAction().getRWheelSpeed()!=0){
            std::cout<<"terminating!"<<std::endl;
            currentTask=Task(Disturbance(), STOP);
            currentTask.getAction().setLWheelSpeed(0);
            currentTask.getAction().setRWheelSpeed(0);
            currentTask.set_change(false);
        }
        return currentTask;
    }

};

class DebugTracker: public ClosedLoop_Tracker{
    void printWindow(){
        b2AABB aabb;
        attention_window.ComputeAABB(&aabb, b2Transform_zero, 0);
        std::cout<<"upperbound="<<aabb.upperBound.x<<", "<<aabb.upperBound.y<<std::endl;
        std::cout<<"lowerbound="<<aabb.lowerBound.x<<", "<<aabb.lowerBound.y<<std::endl;

    }


};


class UserInputDR:public UserInputConfigurator{
    public:
    UserInputDR(DirectionSetter * ds, AffordanceSetter * as): UserInputConfigurator(ds, as){}
    void getTaskFromInput(){
        Disturbance disturbance;
        if (directionSetter->getDirection()==DEFAULT && affordanceSetter->getAffIndex()==AVOID){
            (worldBuilder->get_world_objects()[0]).attention=true;   
        }
        disturbance.bf=worldBuilder->get_world_objects()[0];
        disturbance.set_affordance(affordanceSetter->getAffIndex());
        disturbance.validate();
        if (directionSetter->getDirection()==DEFAULT && affordanceSetter->getAffIndex()==AVOID){
            assignNewGoal(disturbance);    
        }
        Task task(disturbance, directionSetter->getDirection(), b2Transform_zero, true);
        b2World world(GRAVITY);
        worldBuilder->buildWorld(world, b2Transform_zero, task.get_direction(), disturbance);
        if(directionSetter->getDirection()==DEFAULT && affordanceSetter->getAffIndex()==PURSUE){
            task.setEndCriteria(Distance(0.14));
        }

        std::cout<<"goal valid "<<controlGoal.get_disturbance().getAffIndex()<<" pose x="<<controlGoal.get_disturbance().pose().p.x<<" y="<<controlGoal.get_disturbance().pose().p.y<<std::endl;
        simResult sr=simulate(task, world);
        std::cout<<"simulated!"<<sr.step<<" steps"<<std::endl;
        vertexDescriptor v1=boost::add_vertex(transitionSystem);
        auto e=boost::add_edge(currentVertex, v1, transitionSystem);
        transitionSystem[v1].direction=directionSetter->getDirection();
        transitionSystem[e.first].step=sr.step;
        m_plan={v1};
        currentTask.set_change(true);
        transitionSystem[e.first].it_observed=iteration;
    }
};





#endif