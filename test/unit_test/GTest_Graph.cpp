#include "test_classes.h"

class EdgeTest: public Edge, public testing::Test, public testing::WithParamInterface<int>{
};

TEST_P(EdgeTest, override){
    step=GetParam();
    enableOverride();
    if (step==0){
        EXPECT_TRUE(overrideZeroSteps);
    }
    else{
        EXPECT_FALSE(overrideZeroSteps);
    }
}

INSTANTIATE_TEST_CASE_P(Step, EdgeTest, ::testing::Values(0, 1, 200));
 
