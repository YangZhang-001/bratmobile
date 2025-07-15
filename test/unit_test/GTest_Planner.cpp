#include "test_classes.h"


 
TEST_P(ConfiguratorTakeBool, RecyclePlan){
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4};
    vertexDescriptor task_start=DUMMY;
    make_ts(avoid, desiredPlan, GetParam());    
    State s=transitionSystem[2];
    b2Transform shift=b2Mul(transitionSystem[DUMMY].endPose, transitionSystem[currentVertex].endPose);
    b2Transform shift_start=b2Transform_zero;
    math::applyAffineTrans(shift, transitionSystem);
    EXPECT_EQ(transitionSystem[currentVertex].endPose, b2Transform_zero);
    iteration=100;
    resetPhi();
    VertexMatch vm(StateMatcher::ABSTRACT, 2);
    auto edge =boost::add_edge(currentVertex, 2, transitionSystem);
    bool recycled=recycle_plan(currentVertex, currentVertex, task_start, vm.first, shift_start, s.start, edge, m_plan, s.direction);
    EXPECT_TRUE(recycled);
    EXPECT_EQ(m_plan, desiredPlan);
}


TEST_F(ConfiguratorPlannerHybrid,pathToAddTo){
    HorizonStarPlanner horizonPlanner;
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4}, add;
    std::vector<std::vector<vertexDescriptor>> paths;
    paths.emplace_back(std::vector<vertexDescriptor>({14, 1, 3, 4}));
    vertexDescriptor task_start=DUMMY;
    make_ts(avoid, desiredPlan, true); 
    std::vector<std::vector<vertexDescriptor>>::reverse_iterator path=paths.rbegin();
    add={5, 6};
    path2add2(path, add, paths, transitionSystem);
    EXPECT_EQ(paths.size(), 2);
    EXPECT_EQ(paths[1][paths[1].size()-1], 1);
    EXPECT_EQ(paths[1][paths[1].size()-2], 14);
    EXPECT_EQ(*path, paths[1]);
    EXPECT_TRUE(boost::edge(*(path->rbegin()), add[0], transitionSystem).second);
    
}


TEST_P(ConfiguratorTakeBool, startRecycle){
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4}, add;
    std::vector<std::vector<vertexDescriptor>> paths;
    paths.emplace_back(std::vector<vertexDescriptor>({14, 1, 3, 4}));
    vertexDescriptor solution=DUMMY;
    make_ts(avoid, desiredPlan, true); 
    currentVertex=*desiredPlan.rbegin();
    if (!GetParam()){
        transitionSystem[2].outcome=simResult::successful;
        solution=currentVertex;
    }
    EXPECT_EQ(getRecyclingStart(currentVertex,2), solution);
    
}

// TEST_P(ConfiguratorTestPlanner, frontierVertices){
//     dummy_vertex(MOVING_VERTEX);
//     make_module(currentVertex);
//     setAllVisited();
//     assignOutcome();
//     transitionSystem[currentEdge].overrideZeroSteps=true;
//     int expected=n_successful(withDirection(DEFAULT));
//     ExecutionInfo info=package_info();
//     auto frontier=frontierVertices(MOVING_VERTEX, transitionSystem, info);    
//     EXPECT_EQ(frontier.size(), expected);
    
// }

// INSTANTIATE_TEST_CASE_P(FrontierGalore, ConfiguratorTestPlanner, testing::Combine(::testing::Values(0, 1, 2, 3), 
//                                                                 ::testing::Values(LEFT, RIGHT, DEFAULT), 
//                                                                 ::testing::Values(simResult::crashed, simResult::successful, simResult::safeForNow)));





// void ConfiguratorPlannerHybrid::assignOutcome(){
//     Direction direction=std::get<1>(GetParam());
//     int n_assign=std::get<0>(GetParam());
//     std::vector<vertexDescriptor> with_direction=withDirection(direction);
//     simResult::resultType outcome=std::get<2>(GetParam());
//     for (int i=0; i<with_direction.size(); i++){
//         if (i<=n_assign){
//             transitionSystem[with_direction[i]].outcome=outcome;
//         }
//     }
// }