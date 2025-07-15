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

TEST_P(ConfiguratorTakeBool, CheckVectorForPredicate){
    vertexDescriptor v1;
    Edge e;
    bool solution=false; 
    if (GetParam()){
        e.it_observed=2;
        solution=true;
    }
    add_vertex_now(currentVertex, v1, controlGoal.get_disturbance(), e,true);
    SameIteration si(transitionSystem, 2);
    std::vector<edgeDescriptor> ie=inEdges(v1, UNDEFINED);
    auto it=check_vector_for(ie, si);
    bool hasResult=it!=ie.end();
    EXPECT_EQ(hasResult, solution);
    
}