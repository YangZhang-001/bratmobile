#include "test_classes.h"
const bool DEBUG=false;

TEST_F(HighLevelTestBase, TaskToExecNoChangeVertices){
    wc.task_to_execute(configurator->get_plan(), configurator->get_ts(), 0, configurator->getGoal(), configurator->getTask(), configurator->get_current_vertices());
    EXPECT_TRUE(configurator->get_current_vertices().empty());
}


TEST_F(HighLevelTestBase, NextTaskChangeVerticesEmpty){
    wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(),configurator->get_plan_nConst());
    EXPECT_FALSE(configurator->get_current_vertices().empty());
    EXPECT_EQ(configurator->get_current_vertices()[0], MOVING_VERTEX);
}

TEST_F(HighLevelTestBase, NextTaskChangeVerticesPlan){
    configurator->make_module(MOVING_VERTEX);
    configurator->set_plan({2, 3});
    wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(),configurator->get_plan_nConst());
    EXPECT_EQ(configurator->get_current_vertices(), std::vector<vertexDescriptor>({2}));
}

/**
 * Check if current_vertices is dummy vertex after next_task when plan is empty
 */
TEST_F(HighLevelTestBase, NextTaskChangeVerticesDummy){
    configurator->dummy_vertex(MOVING_VERTEX);
    wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(),configurator->get_plan_nConst());
    EXPECT_EQ(configurator->get_current_vertices(), std::vector<vertexDescriptor>({0}));

}