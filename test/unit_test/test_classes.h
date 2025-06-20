#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"
#include <string>

/**
 * @brief Setting up ostream operator for use with GTest
 * 
 * @param os 
 * @param t 
 * @return std::ostream& 
 */
std::ostream& operator<<(std::ostream& os, const b2Transform& t){
    os << "b2Transform(pos=("
       << t.p.x << ", " << t.p.y << "), rot=("
       << t.q.c << ", " << t.q.s << "))";
    return os;
}

class DebugConfigurator:public Configurator{
    friend class HighLevelTest;
    public:
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

    void clear_plan(){
        plan.clear();
    }


    void vertex_set_state(vertexDescriptor v, const State &s){
        transitionSystem[v]=s;
    }

    void vertex_set_endPose(vertexDescriptor v, b2Transform t){
        transitionSystem[v].endPose=t;
    }

    void vertex_set_start(vertexDescriptor v, b2Transform t){
        transitionSystem[v].start=t;
    }

    void vertex_set_direction(vertexDescriptor v, Direction d){
        transitionSystem[v].direction=d;
    }

    void vertex_set_outcome(vertexDescriptor v, simResult::resultType r){
        transitionSystem[v].outcome=r;
    }

    void vertex_set_Di(vertexDescriptor v, const Disturbance & d){
        transitionSystem[v].Di=d;
    }

    void vertex_set_Dn(vertexDescriptor v, const Disturbance & d){
        transitionSystem[v].Dn=d;
    }

    void vertex_set_options(vertexDescriptor v, const std::vector <Direction> & d){
        transitionSystem[v].options=d;
    }

    const State & vertex_get_state(vertexDescriptor v){
        return transitionSystem[v];
    }

    b2Transform vertex_get_endPose(vertexDescriptor v){
        return transitionSystem[v].endPose;
    }

    b2Transform vertex_get_start(vertexDescriptor v){
        return transitionSystem[v].start;
    }

    Direction vertex_get_direction(vertexDescriptor v){
        return transitionSystem[v].direction;
    }

    simResult::resultType vertex_get_outcome(vertexDescriptor v){
        return transitionSystem[v].outcome;
    }

    const Disturbance & vertex_get_Di(vertexDescriptor v){
        return transitionSystem[v].Di;
    }

    const Disturbance & vertex_get_Dn(vertexDescriptor v){
        return transitionSystem[v].Dn;
    }

    const std::vector <Direction>& vertex_get_options(vertexDescriptor v){
        transitionSystem[v].options;
    }
    

    vertexDescriptor get_current_vertex(){
        return currentVertex;
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
    Motor_Out m;

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
     * 
     */
    std::vector<vertexDescriptor> get_plan(std::string folder);


    

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
    // int desired_split_size(b2Vec2 pos, float simulationStep){
    //     return int(pos.Length()/(simulationStep+0.00001))+1;
    // }
    // /**
    //  * @brief Tests task split function
    //  * 
    //  * @param x coordinate of robot
    //  * @param y coordinate of robot
    //  * @param th coordinate of robot
    //  * @param Dx coordinate of Dn
    //  * @param Dy coordinate of Dn
    //  * @param Dth coordinate of Dn
    //  * @return std::vector <vertexDescriptor> 
    //  */
    // std::vector <vertexDescriptor> test_split(float x, float y, float th, float Dx, float Dy, float Dth){
    //     b2Transform start=transitionSystem[movingVertex].endPose;
    //     auto v1 = boost::add_vertex(transitionSystem);
    //     auto e1 = boost::add_edge(currentVertex, v1, transitionSystem);
    //     b2Vec2 pos(x,y);
    //     b2Rot rot(th);
    //     transitionSystem[v1].direction=DEFAULT;
    //     transitionSystem[v1].start=start;
    //     transitionSystem[v1].outcome=simResult::crashed;    
    //     transitionSystem[v1].endPose=b2Transform(pos, rot);
    //     transitionSystem[v1].Dn=Disturbance(AVOID,  b2Vec2(Dx, Dy), Dth);
    //     return splitTask(v1, transitionSystem, transitionSystem[v1].direction, currentVertex);
    // }


};


/**
 * @brief Configurator fixture parametrised for b2Transforms
 * 
 */
class ConfiguratorTest2DT:public ConfiguratorTest, public testing::WithParamInterface<b2Transform>{

};

bool DebugConfigurator::plan_reaches_horizon(){
    return fabs(plan_end_b2Vec2().Length()-BOX2DRANGE)<0.02;
}

bool DebugConfigurator::plan_reaches_goal(){
    return (plan_end_b2Vec2()-controlGoal.disturbance.pose().p).Length()<0.02;
}

void HighLevelTest::init( const Task& goal){
    configurator->init(goal);
    configurator->currentTask.set_change(true);
    configurator->register_controller(&wc);
    configurator->register_tracker(&tracker);
    configurator->registerInterface(&ci, &m);
    configurator->setSimulationStep(ROBOT_HALFWIDTH*2);
}


std::vector<vertexDescriptor> HighLevelTest::get_plan(std::string folder){
    DataInterface di(&ci);
    di.set_folder(folder);
    di.newScanAvail();
    configurator->data2fp= ci.data2fp;
    configurator->Spawner();
    return configurator->get_plan();
}
#endif