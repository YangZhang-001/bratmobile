#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"

//using ::testing::Test;  // GTest test fixture
 /**
 * @brief Test fixture for Configurator
 * 
 */
class ConfiguratorTest: public ::testing::Test{
    protected:
    Configurator configurator;
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    LIDAR_In ci;
    Motor_Out m;

    void SetUp()override{}

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
        configurator.simulationStep=ROBOT_HALFWIDTH*2;
        try{
            configurator.Spawner();
        }
        catch(std::exception &e){
            std::cerr<<e.what()<<std::endl;
        }
        return configurator.plan;
    }


int desired_split_size(b2Vec2 pos, float simulationStep){
    return int(pos.Length()/(simulationStep+0.00001))+1;
}
    

};
#endif