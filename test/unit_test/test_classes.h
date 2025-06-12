#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"

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



//using ::testing::Test;  // GTest test fixture
 /**
 * @brief Test fixture for testing high-level processes such as planning and state-space exploration
 * 
 */
class HighLevelTest: public ::testing::Test{
    protected:
    Configurator configurator;
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    LIDAR_In ci;
    Motor_Out m;

    void SetUp()override{
        //configurator.transitionSystem.m_edges.clear();
        // configurator.transitionSystem.m_vertices.clear();
        // boost::clear_vertex(configurator.movingVertex, configurator.transitionSystem);

    }

    void TearDown()override{}
    /**
     * @brief Initialises Fixture
     * 
     * @param goal overarching goal
     */
    void init( Task& goal){
        configurator.init(goal);
        configurator.register_controller(&wc);
        configurator.register_tracker(&tracker);
        configurator.registerInterface(&ci, &m);
        configurator.simulationStep=ROBOT_HALFWIDTH*2;
    }
    /**
     * @brief Tests planning
     * 
     * @param folder a folder containing LIDAR scans names "map%04i.dat"
     * 
     */
    std::vector<vertexDescriptor> get_plan(char * folder){
        DataInterface di(&ci);
        di.folder=folder;
        di.newScanAvail();
        configurator.data2fp= ci.data2fp;
        try{
            bool edges=configurator.transitionSystem.m_edges.empty();
            configurator.Spawner();
        }
        catch(std::exception &e){
            std::cerr<<e.what()<<std::endl;
        }
        return configurator.plan;
    }


    

};

/**
 * @brief Fixture class for testing Configurator functions
 */
class ConfiguratorTest: public Configurator, public testing::Test{ //, testing::TestWithParam<float>
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



#endif