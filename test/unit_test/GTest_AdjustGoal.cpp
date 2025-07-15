#include "test_classes.h"
#include <gtest/gtest.h>

/**
 * @brief Tests whether the tracked disturbance presents an unexpected shitf, the robot is able to track the goal assuming constant relationships between obstacle and goal
 * 
 */
TEST_P(ConfiguratorTest2DT, adjustGoal){
    /**
     * @brief Setup: robot drives towards an obstacle and has a target position behind it
     * the obstacle shifts unpredictably and the robot has to adjust the expectation of the goal,
     * assuming that the ratio between obstacle and goal is constant
     * 
     */
    Task goal(Disturbance(PURSUE, b2Vec2(1.0, 0)), DEFAULT);
    Disturbance obstacle(PURSUE, b2Vec2(0.45, 0), 0); //robot is driving towards an obstacle before it avoids it
    init(goal);
    currentTask=Task(obstacle, DEFAULT, b2Transform_zero,true);
    b2Transform deltaPose=GetParam();
    ConfiguratorTest::Manual_WiseController controller;
    register_controller(&controller);
    controller.set_disturbance(obstacle);
    controller.set_Di_to_goal(goal.get_disturbance());
    vertexDescriptor v1;
    vertex_set_options(0, std::vector<Direction>(DEFAULT));
    add_vertex_now(MOVING_VERTEX, v1, goal.get_disturbance());
    m_plan={v1};
    math::applyAffineTrans(deltaPose, deltaPose);
    update_graph(transitionSystem, deltaPose);
    b2Transform expected =b2MulT(controller.get_disturbance().pose(), transitionSystem[plan_end()].Di.pose()); //position of goal wrt current disturbance
    /**/
    adjust_goal_expectation(); //what we're actually testing
    b2Transform observed =b2MulT(currentTask.get_disturbance().pose(), controlGoal.get_disturbance().pose());
    b2Transform difference=expected-observed;
    EXPECT_LT(difference.p.Length(),0.001);
    EXPECT_LT(fabs(difference.q.GetAngle()),0.001);

}

INSTANTIATE_TEST_CASE_P(AdjustGoal, 
                        ConfiguratorTest2DT, 
                        ::testing::Values(b2Transform(b2Vec2(0, 0), b2Rot(0)),
                                          b2Transform(b2Vec2(0, 0), b2Rot(M_PI_2)), 
                                          b2Transform(b2Vec2(0, 0), b2Rot(-0.23)),
                                          b2Transform(b2Vec2(0.4, 0), b2Rot(0)),
                                          b2Transform(b2Vec2(0, 0.3), b2Rot(0)),
                                          b2Transform(b2Vec2(-0.4, 0.3), b2Rot(0)),
                                          b2Transform(b2Vec2(0, -0.3), b2Rot(0.12)),
                                          b2Transform(b2Vec2(-0.4, 0), b2Rot(0.12)),
                                          b2Transform(b2Vec2(0.8, -0.05), b2Rot(-0.45))));

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}