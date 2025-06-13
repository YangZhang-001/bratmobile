#include "test_classes.h"
#include <gtest/gtest.h>

TEST(Initialisation, Constructor){
    DebugConfigurator configurator();
}

TEST(Initialisation, InitialMap){
    DebugConfigurator configurator;
    EXPECT_EQ(configurator.n_vertices(),1);
    EXPECT_EQ(configurator.n_edges(), 0);
}