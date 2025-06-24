#include "test_classes.h"
#include <gtest/gtest.h>


TEST(Initialisation, DebugConstructor){
    DebugConfigurator configurator;
}

TEST_F(ConfiguratorTest, Initialisation){
}

TEST(Initialisation, Task){
    Task task;
}

TEST(Initialisation, InitialMap){
    DebugConfigurator configurator;
    EXPECT_EQ(configurator.n_vertices(),1);
    EXPECT_EQ(configurator.n_edges(), 0);
}

TEST_F(ConfiguratorTest, InitialVertex){
    EXPECT_EQ(currentVertex, movingVertex);
}

TEST_F(ConfiguratorTest, DummyVertex){
    init();
    dummy_vertex(movingVertex);
    EXPECT_EQ(boost::out_degree(movingVertex, transitionSystem), 1);
    EXPECT_FALSE(boost::edge(movingVertex, movingVertex, transitionSystem).second);
}

/**
 * @brief Parameters: robot position, previous task direction, current task direction
 * 
 */
class ConfiguratorTestGetGoal:public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, Direction,Direction>>{
    protected:
    /**
     * @brief Sets up vertex for testing using the parameters
     * 
     * @param v 
     */
    void vertex_setup(vertexDescriptor v){
        transitionSystem[v].direction=std::get<1>(GetParam());
        transitionSystem[v].endPose=std::get<0>(GetParam());
        vertex_options_push_back(v, std::get<2>(GetParam()));
    }
};

TEST_P(ConfiguratorTestGetGoal, GetDisturbanceGoal){
    Disturbance solution(PURSUE, b2Vec2(1.0, 0));
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    bf.attention=1;
    data2fp.emplace(bf.pose.p); //make point corresponding to obstacle
    transitionSystem[movingVertex].Di=Disturbance(bf); //current task was avoiding
    transitionSystem[movingVertex].Di.validate();
    vertex_setup(movingVertex);
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, std::get<2>(GetParam()), b2Transform_zero);
    EXPECT_TRUE(Di==solution);
}

INSTANTIATE_TEST_CASE_P(DisturbanceIsGoal, ConfiguratorTestGetGoal, ::testing::Values(
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), DEFAULT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), DEFAULT, RIGHT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), LEFT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), RIGHT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, RIGHT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, LEFT)));

class ConfiguratorTestGetObstacle: public ConfiguratorTestGetGoal{};

TEST_P(ConfiguratorTestGetObstacle, GetDisturbanceObstacle){
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    bf.attention=1;
    Disturbance solution(bf);
    data2fp.emplace(bf.pose.p); //make point corresponding to obstacle
    transitionSystem[movingVertex].Di=solution; //current task was avoiding
    transitionSystem[movingVertex].Di.validate();
    vertex_setup(movingVertex);
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, std::get<2>(GetParam()), b2Transform_zero);
    EXPECT_TRUE(Di==solution);
}

INSTANTIATE_TEST_CASE_P(DisturbanceIsObstacle, ConfiguratorTestGetGoal, ::testing::Values(
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.4, 0.0), b2Rot(M_PI_2)), LEFT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.4, 0.0), b2Rot(M_PI_2)), RIGHT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.40, 0.31), b2Rot(-M_PI_2)), RIGHT, DEFAULT)));

TEST_F(ConfiguratorTest, GetDisturbanceObstacle2){
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    bf.attention=1;
    data2fp.emplace(bf.pose.p); //make point corresponding to obstacle
    transitionSystem[movingVertex].Di=Disturbance(PURSUE, b2Vec2(1.0, 0)); //current task was avoiding
    transitionSystem[movingVertex].Dn=Disturbance(bf); //current task was avoiding
    transitionSystem[movingVertex].Dn.validate();
    Disturbance solution=transitionSystem[movingVertex].Dn;
    transitionSystem[movingVertex].direction=DEFAULT;
    transitionSystem[movingVertex].endPose.p.x=0.4;
    vertex_options_push_back(movingVertex, LEFT);
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, LEFT, b2Transform_zero);
    EXPECT_TRUE(Di==solution);
}

TEST_F(ConfiguratorTest, GetDisturbanceObstacle3){
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    bf.attention=1;
    data2fp.emplace(bf.pose.p); //make point corresponding to obstacle
    transitionSystem[movingVertex].Dn=Disturbance(bf); //current task was avoiding
    transitionSystem[movingVertex].Dn.validate();
    Disturbance solution=transitionSystem[movingVertex].Dn;
    transitionSystem[movingVertex].direction=DEFAULT;
    transitionSystem[movingVertex].endPose.p.x=0.4;
    vertex_options_push_back(movingVertex, LEFT);
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, LEFT, b2Transform_zero);
    EXPECT_TRUE(Di==solution);
}