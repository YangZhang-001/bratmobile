#include "test_classes.h"
#include <gtest/gtest.h>

//for me because I never remember linear algebra
class LinearTransformationTest: public ::testing::TestWithParam<float>{};

TEST_P(LinearTransformationTest, rotateVec2){
    float angle=GetParam(), radius=.27;
    b2Rot rot(angle);
    b2Vec2 vec(radius*rot.c, radius*rot.s), vec2(.27, 0), vec3=b2Mul(rot, vec2);
    EXPECT_EQ(vec.x, vec3.x);
    EXPECT_EQ(vec.y, vec3.y);
}   

TEST_P(LinearTransformationTest, rotateTransform){
    float angle=GetParam(), radius=.27;
    b2Rot rot(angle);
    b2Vec2 vec(radius*rot.c, radius*rot.s), vec2(.27, 0);//, vec3=b2Mul(rot, vec2);
    b2Transform t1(vec, b2Rot(0)), t2(vec2, b2Rot(0)),t3(b2Vec2(0.0, 0.5), rot), tMul=b2Mul( t3, t2), tAdd=t3+t1, tSub=t3-t1, tNeg=InvMul(t2, t2);
    EXPECT_EQ(tAdd.p.x, tMul.p.x);
    EXPECT_EQ(tAdd.p.y, tMul.p.y);
    EXPECT_EQ(tAdd.q.GetAngle(), tMul.q.GetAngle());
    EXPECT_EQ(tAdd.q.GetAngle(), tMul.q.GetAngle());

    EXPECT_EQ(tSub.p.x, tNeg.p.x);
    EXPECT_EQ(tSub.p.y, tNeg.p.y);
    EXPECT_EQ(tSub.q.GetAngle(), tNeg.q.GetAngle());
    EXPECT_EQ(tSub.q.GetAngle(), tNeg.q.GetAngle());
}  

INSTANTIATE_TEST_CASE_P(Angles, LinearTransformationTest, testing::Values(0, M_PI_4, M_PI/6, M_PI_2, M_PI_2+M_PI/6,-M_PI_4, -M_PI/6, -M_PI_2, -M_PI_2-M_PI/6 ));