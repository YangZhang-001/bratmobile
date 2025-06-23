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