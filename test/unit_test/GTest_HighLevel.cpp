#include "test_classes.h"
#include <gtest/gtest.h>

TEST_F(HighLevelTest, Init){
    init(Task());
    EXPECT_TRUE(configurator.get_motor_interface()!=(NULL));
    EXPECT_TRUE(configurator.get_lidar_interface()!= NULL);
    EXPECT_TRUE(configurator.get_tracker()!=NULL);
    EXPECT_TRUE(configurator.get_controller()!=NULL);

}

TEST_P(HighLevelTest, Plan){
    Task goal;
    if (GetParam().first){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);

    }
    init(goal);
    get_plan(GetParam().second);
    vertexDescriptor planEnd=configurator.get_plan()[configurator.get_plan().size()-1];
    float difference;
    if (!GetParam().first){
        difference=BOX2DRANGE-configurator.transitionSystem[planEnd].endPose.p.Length();
    }
    else{
        difference=(configurator.transitionSystem[planEnd].endPose.p-goal.disturbance.pose().p).Length();
    }
    EXPECT_TRUE(configurator.get_plan().size()!=0);
    EXPECT_LT(fabs(difference), 0.03);
}

INSTANTIATE_TEST_CASE_P(FormPlan, HighLevelTest, ::testing::Values(std::pair(false, std::string()),
                                                                   std::pair(false, std::string("../cul_de_sac/")),
                                                                   std::pair (true, std::string("../target_40cm/")),
                                                                   std::pair (true, std::string("../target_68cm/")),
                                                                   std::pair (true, std::string("../cul_de_sac/"))));

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