#include "test_classes.h"
#include <gtest/gtest.h>

TEST_P(ConfiguratorTest32DT, backtracksimple){
        //make three branches (all default, one crashed the other two successful)
        auto v1=make_v1_crashed().m_target;
        auto v2=make_successful().m_target;
        auto v3=make_successful().m_target;
        //make teh other two vertices rotations(just to give them a lower phi value)
        transitionSystem[v2].endPose.q.Set(M_PI_2);
        transitionSystem[v3].endPose.q.Set(-M_PI_2);
        std::vector <vertexDescriptor> evaluationQ={v1, v2, v3}, all=evaluationQ, priorityQ;
        std::set <vertexDescriptor> closed;
        //backtrack
        backtrack(evaluationQ, priorityQ, closed, plan);
        for (vertexDescriptor v:all){
            EXPECT_TRUE(transitionSystem[v].visited()); //check vertex marked as visited
        }
        EXPECT_NE(priorityQ[0], v1); //check that v1 is not the vertex with highest priority
}


INSTANTIATE_TEST_CASE_P(Backtrack, ConfiguratorTest32DT, ::testing::Values(std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.6, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0, 0), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.26, -0.01), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.265, -0.16), b2Rot(0)))));