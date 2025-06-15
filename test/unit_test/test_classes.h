#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"
#include <string>

/**
 * @brief Setting up ostream operator for use with GTest
 * 
 * @param os ostream
 * @param t box2d 2dtransform
 * @return std::ostream& 
 */
std::ostream& operator<<(std::ostream& os, const b2Transform& t){
    os << "b2Transform(pos=("
       << t.p.x << ", " << t.p.y << "), rot=("
       << t.q.c << ", " << t.q.s << "))";
    return os;
}

class DebugConfigurator:public AttentiveConfigurator{
    public:
    friend class HighLevelTest;
    int n_edges(){return transitionSystem.m_edges.size();}

    int n_vertices(){return transitionSystem.m_vertices.size();}

    const std::vector <vertexDescriptor>& get_plan(){ return plan;}

    bool plan_reaches_horizon();

    bool plan_reaches_goal();

    vertexDescriptor plan_end(){return plan[plan.size()-1];}

    b2Vec2 plan_end_b2Vec2(){return transitionSystem[plan_end()].endPose.p;}

    Task & getTask(){ //returns Task being executed
        return currentTask;
    }

    std::vector <BodyFeatures> & world_objects(){
        return worldBuilder.get_world_objects();
    }

    TransitionSystem & get_ts(){
        return transitionSystem;
    }

    Task & getGoal(){
        return controlGoal;
    }

    void set_data2fp(const CoordinateContainer &data){
        data2fp=data;
    }

    int data_size(){
        return data2fp.size();
    }
};


 /**
 * @brief Test fixture for testing high-level processes such as planning and state-space exploration
 * 
 * @param bool does plan have a target location
 * @param string the folder with the LIDAR scans
 */
class HighLevelTest: public testing::Test, public testing::WithParamInterface<std::pair<bool, std::string>>{
    protected:


    DebugConfigurator * configurator=NULL;
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    LIDAR_In ci;
    DataInterface di;
    Motor_Out m;
    HorizonStarPlanner planner;
    int iteration=0;
    void SetUp()override{
        configurator=new DebugConfigurator();
        init();
    }

    void TearDown()override{
        try{
            delete configurator;
        }
        catch(...){
            std::cout <<"caught!"<<std::endl;
        }
    }
    /**
     * @brief Initialises Fixture
     * 
     * @param goal overarching goal
     */
    void init( const Task& goal=Task());
    /**
     * @brief Tests planning
     * 
     * @param folder a folder containing LIDAR scans names "map%04i.dat"
     * @param it iteration of data interface (determines which map will be read) - 0 reads map 1
     * 
     */
    std::vector<vertexDescriptor> get_plan(std::string folder, int it=0);
    public:

    HighLevelTest(){}

    

};

/**
 * @brief Fixture class for testing Configurator functions
 */
class ConfiguratorTest: public DebugConfigurator, public testing::Test{ //, testing::TestWithParam<float>
protected:
    /**
     * @brief Allows to set parameters manually from the Configurator
     * 
     */
    class Manual_WiseController:public Wise_Controller{
        public:
        /**
         * @brief Manually set the distance of Di(disturbance_q) to goal
         * 
         * @param goal goal disturbance
         */
        void set_Di_to_goal(const Disturbance & goal){
            _D_to_goal=b2MulT(disturbance_q.pose(), goal.pose());
        }
    };


public:
    /**
     * @brief Manually set options for a state transition
     * 
     * @param v the index of the state
     * @param options desired options vector
     */
    void graph_setOptions(vertexDescriptor v, const std::vector<Direction> & options){
        transitionSystem[v].options=options;
    }


};


/**
 * @brief Configurator fixture parametrised for b2Transforms
 * 
 */
class ConfiguratorTest2DT:public ConfiguratorTest, public testing::WithParamInterface<b2Transform>{

};

/**
 * @brief Configurator parameters are 2 2dtransforms
 * 
 */
class ConfiguratorTest32DT:public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, b2Transform,b2Transform>>{
    protected:
    ConfiguratorTest32DT(){
        register_tracker(new ClosedLoop_Tracker);
    }

    ~ConfiguratorTest32DT(){
        delete tracker;
    }
    
    /**
     * @brief Creates a vertex whose state starts and end at the origin
     * 
     * @param v0 
     * @return edgeDescriptor 
     */
    edgeDescriptor make_successful(vertexDescriptor v0=0);
    /**
     * @brief returns an edge connecting vertex v0 to a vertex pointing to a crashed state
     * 
     */
    edgeDescriptor make_v1_crashed( vertexDescriptor v0=0);

    public:
        void SetUp(){
        transitionSystem=TransitionSystem(1);
    }

    void TearDown(){
        transitionSystem.clear();
    }
};


bool DebugConfigurator::plan_reaches_horizon(){
    return fabs(plan_end_b2Vec2().Length()-BOX2DRANGE)<0.02;
}

bool DebugConfigurator::plan_reaches_goal(){
    return (plan_end_b2Vec2()-controlGoal.disturbance.pose().p).Length()<0.02;
}

void HighLevelTest::init( const Task& goal){
    di.registerInterface(&ci);
    configurator->register_controller(&wc);
    configurator->register_tracker(&tracker);
    configurator->registerInterface(&ci, &m);
    configurator->setSimulationStep(ROBOT_HALFWIDTH*2);
    configurator->register_planner(&planner);
    configurator->currentTask.set_change(true);
    configurator->init(goal);

}


std::vector<vertexDescriptor> HighLevelTest::get_plan(std::string folder, int it){
    di.set_iteration(it);
    di.set_folder(folder);
    di.newScanAvail();
    configurator->data2fp= ci.data2fp;
    configurator->Spawner();
    return configurator->get_plan();
}

edgeDescriptor ConfiguratorTest32DT::make_successful(vertexDescriptor v0){
    auto v1=boost::add_vertex(transitionSystem);
    auto e=boost::add_edge(v0, v1, transitionSystem);
    transitionSystem[v1].direction=DEFAULT;
    transitionSystem[e.first].step=1;
    return e.first;
}

edgeDescriptor ConfiguratorTest32DT::make_v1_crashed( vertexDescriptor v0){
    edgeDescriptor e=make_successful(v0);
    vertexDescriptor v1=e.m_target;
    b2Transform Dn=std::get<2>(GetParam());
    transitionSystem[v1].outcome=simResult::crashed;
    transitionSystem[v1].start=std::get<0>(GetParam()); //start
    transitionSystem[v1].endPose=std::get<1>(GetParam());//pose
    transitionSystem[v1].Dn=Disturbance(AVOID, Dn.p,Dn.q.GetAngle());
    return e;
}
#endif