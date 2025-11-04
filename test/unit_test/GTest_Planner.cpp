#include "test_classes.h"


TEST_F(ConfiguratorPlannerHybrid,pathToAddTo){
    //HorizonStarPlanner horizonPlanner;
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

/**
 * @brief This test is made to recreate a situation in which the planner gets stuck in 
 * an endless loop due to a self-edge, recreating error so that it can be fixed
 */
TEST_F(ConfiguratorPlannerHybrid, GetUnStuck){
    init(generateGoalTask());
    dummy_vertex(MOVING_VERTEX);
    auto e=make_successful(currentVertex);
    add_edge_withPoses(e.m_target, e.m_target); //self-edge
    auto self_edge= boost::edge(e.m_target, e.m_target, transitionSystem).first;
    transitionSystem[self_edge].step=0;
    transitionSystem[self_edge].overrideZeroSteps=true;
    make_module(e.m_target); //vertices 3-7
    boost::clear_vertex(3, transitionSystem);
    ExecutionInfo execInfo=package_info();
    setAllVisited();
    vertex_set_phi(e.m_target, .0f);
    auto fv=frontierVertices(e.m_target, transitionSystem, execInfo);
    EXPECT_NE(fv.size(), 0);
}