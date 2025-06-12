#include "test_classes.h"
#include <gtest/gtest.h>


TEST_F(HighLevelTest, planToTarget1){
    Task goal(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    init(goal);
    std::vector<vertexDescriptor> plan=get_plan("../target_68cm/");
    vertexDescriptor planEnd=configurator.plan[configurator.plan.size()-1];
    b2Vec2 difference=configurator.transitionSystem[planEnd].endPose.p-goal.disturbance.pose().p;
    EXPECT_TRUE(plan.size()!=0);
    EXPECT_LT(difference.Length(), 0.03);
}

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

TEST_P(ConfiguratorTest2DT, adjustGoal){
    /**
     * @brief Setup: robot drives towards an obstacle and has a target position behind it
     * the obstacle shifts unpredictably and the robot has to adjust the expectation of the goal,
     * assuming that the ratio between obstacle and goal is constant
     * 
     */
    Task goal(Disturbance(PURSUE, b2Vec2(1.0, 0)), DEFAULT);
    Disturbance obstacle(PURSUE, b2Vec2(0.5, 0), 0); //robot is driving towards an obstacle before it avoids it
    init(goal);
    currentTask.disturbance=obstacle;
    b2Transform deltaPose=GetParam();
    ConfiguratorTest::Manual_WiseController controller;
    register_controller(&controller);
    vertexDescriptor v1;
    graph_setOptions(0, std::vector<Direction>(DEFAULT));
    add_vertex_now(movingVertex, v1, transitionSystem, goal.disturbance);
    plan={v1};
    update_graph(transitionSystem, deltaPose);
    vertexDescriptor plan_end=plan[plan.size()-1];
    b2Transform expected =b2MulT(controller.get_disturbance().pose(), transitionSystem[plan_end].Di.pose()); //position of goal wrt current disturbance
    /**/
    adjust_goal_expectation();
    b2Transform observed =b2MulT(currentTask.disturbance.pose(), controlGoal.disturbance.pose());
    b2Transform difference=expected-observed;
   // EXPECT_LT(difference.p.Length(),0.001);
   // EXPECT_LT(fabs(difference.q.GetAngle()),0.001);

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