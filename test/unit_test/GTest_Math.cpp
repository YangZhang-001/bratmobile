#include "test_classes.h"
#include <gtest/gtest.h>

class LinearTransformationTest: public ::testing::TestWithParam<float>{};

TEST_P(LinearTransformationTest, rotateVec2){
    float angle=GetParam(), radius=.27;
    b2Rot rot(angle);
    b2Vec2 vec(radius*rot.c, radius*rot.s), vec2(.27, 0), vec3=b2Mul(rot, vec2);
    EXPECT_EQ(vec.x, vec3.x);
    EXPECT_EQ(vec.y, vec3.y);
}   

INSTANTIATE_TEST_CASE_P(Angles, LinearTransformationTest, testing::Values(0, M_PI_4, M_PI/6, M_PI_2, M_PI_2+M_PI/6,-M_PI_4, -M_PI/6, -M_PI_2, -M_PI_2-M_PI/6 ));