#include <gtest/gtest.h>
#include "../callbacks.h"

 /**
 * @brief Class for testing Configurator
 * 
 */
class Configurator_Test: public ::testing::Test{
    Configurator configurator;
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    LIDAR_In ci;
    Motor_Out m;
    public:

    Configurator_Test(Task * goal);
    
    /**
     * @brief Tests planning
     * 
     * @param folder a folder containing LIDAR scans names "map%04i.dat"
     * 
     */
    std::vector<vertexDescriptor> get_plan(char * folder);

};