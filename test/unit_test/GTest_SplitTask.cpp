#include "test_classes.h"
#include <gtest/gtest.h>

/**
 * @brief Configurator parameters are 2 2dtransforms
 * 
 */
class ConfiguratorTest32DT:public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, b2Transform,b2Transform>>{

};


TEST_P(ConfiguratorTest32DT, splitTask){
    auto v1=boost::add_vertex(transitionSystem);
    auto e=boost::add_edge(movingVertex, v1, transitionSystem);
    transitionSystem[v1].outcome=simResult::crashed;
    b2Transform start=std::get<0>(GetParam());
    transitionSystem[v1].start=start; //start
    transitionSystem[v1].endPose=std::get<1>(GetParam());//pose
    transitionSystem[v1].Dn=Disturbance(AVOID, (std::get<2>(GetParam())).p,(std::get<2>(GetParam()).q.GetAngle()));
    std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem, transitionSystem[v1].direction, currentVertex);
    b2Vec2 endPosition=std::get<1>(GetParam()).p;
    int expected_splitSize=int(endPosition.Length()/(simulationStep+0.00001))+1;
    EXPECT_EQ(split.size(), expected_splitSize);
    int ct=0;
    for (vertexDescriptor v:split){
        float step_size=(transitionSystem[v].endPose.p-start.p).Length();
        EXPECT_LT(step_size, simulationStep+0.00001);
        EXPECT_FALSE(transitionSystem[v].Di.isValid());
        if(ct<(split.size()-1)){
            EXPECT_EQ(transitionSystem[v].outcome, simResult::safeForNow);
        }
        if (ct==(split.size()-1)){
            EXPECT_EQ(transitionSystem[v].outcome, simResult::crashed);
        }

    }
    
}

INSTANTIATE_TEST_CASE_P(SplitTask, ConfiguratorTest32DT, ::testing::Values(std::tuple(b2Transform_zero, b2Transform(b2Vec2(0.6, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple(b2Transform(b2Vec2(0, 0), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.26, -0.01), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.265, -0.16), b2Rot(0))),
                                                                   std::tuple(b2Transform_zero, b2Transform(b2Vec2(0, 0.6), b2Rot(M_PI_2)), b2Transform_zero),
                                                                   std::tuple(b2Transform(b2Vec2(0.2, 0.27), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.75, 0.27), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple(b2Transform(b2Vec2(0.27, 0.2), b2Rot(M_PI_2), b2Transform(b2Vec2(.27, 0.75), b2Rot(M_PI_2)), b2Transform_zero),
                                                                   std::tuple(b2Transform_zero, b2Transform(b2Vec2(0.27, 0), b2Rot(0)), b2Transform_zero)),
                                                                   std::tuple(b2Transform_zero, b2Transform(b2Vec2(0.18, 0), b2Rot(0)), b2Transform_zero)) 
                                                                   );
