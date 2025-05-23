#include "test_classes.h"
#include <gtest/gtest.h>


TEST_F(ConfiguratorTest, planToTarget1){
    Task goal(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    init(goal);
    std::vector<vertexDescriptor> plan=get_plan("../target_68cm/");
    EXPECT_FALSE(plan.empty());
    EXPECT_TRUE(plan.size()!=0);
    // EXPECT_LT(configurator.)
}



int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}