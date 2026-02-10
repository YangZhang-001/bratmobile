#include "test_classes.h"
const bool DEBUG=false;

class EdgeTest: public Edge, public testing::Test, public testing::WithParamInterface<int>{
};

TEST(IsNotV, RemoveEdge){
    is_not_v nv(MOVING_VERTEX);
    TransitionSystem ts(1);
    if (boost::out_degree(MOVING_VERTEX, ts)>0){
        boost::remove_out_edge_if(MOVING_VERTEX,nv, ts);
    }
}

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
 
