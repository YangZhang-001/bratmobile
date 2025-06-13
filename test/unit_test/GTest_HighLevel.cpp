#include "test_classes.h"
#include <gtest/gtest.h>

TEST_P(HighLevelTest, Plan){
    Task goal(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    init(goal);
    DataInterface di(&ci);
    di.folder=GetParam().c_str();
    di.newScanAvail();
    configurator.data2fp= ci.data2fp;
    configurator.Spawner();
    vertexDescriptor planEnd=configurator.plan[configurator.plan.size()-1];
    b2Vec2 difference=configurator.transitionSystem[planEnd].endPose.p-goal.disturbance.pose().p;
    EXPECT_TRUE(configurator.plan.size()!=0);
    EXPECT_LT(difference.Length(), 0.03);
}

INSTANTIATE_TEST_CASE_P(FormPlan, HighLevelTest, ::testing::Values(std::string("../target_68cm/"), std::string("../target_40cm/")));

/**
 * @brief Tests split steps
 * 
 */
// TEST_P(ConfiguratorTest, split_size){
//     b2Vec2 pos(GetParam(), GetParam());
//     int desired=desired_split_size(pos, simulationStep);
//     std::vector <vertexDescriptor> split=test_split(pos.x, pos.y, GetParam(), GetParam(), GetParam(), GetParam())
//     EXPECT_EQ(split.size(), desired);
// }


// TEST_P(ConfiguratorTest, split_size){
//     b2Vec2 pos(GetParam(), GetParam());
//     float max_step=0;
//     std::vector <vertexDescriptor> split=test_split(pos.x, pos.y, GetParam(), GetParam(), GetParam(), GetParam());
//     for (vertexDescriptor v:split){
//         if (float length=(transitionSystem[v].endPose.p-start.p).Length(); length>max_step){
//             max_step=length;
//         }
//     }
//     EXPECT_LT(max_step, conf.simulationStep+0.00001);
// }


int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}