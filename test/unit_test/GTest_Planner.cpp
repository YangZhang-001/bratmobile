#include "test_classes.h"
const bool DEBUG=false;

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