#include "test_classes.h"
#include <gtest/gtest.h>
const bool DEBUG=false;

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
    b2Transform t1(vec, b2Rot(0)), t2(vec2, b2Rot(0)),t3(b2Vec2(0.0, 0.5), rot), tMul=b2Mul( t3, t2), tAdd=t3+t1; // tSub=t3-t1, tNeg=InvMul(t2, t2)
    EXPECT_EQ(tAdd.p.x, tMul.p.x);
    EXPECT_EQ(tAdd.p.y, tMul.p.y);
    EXPECT_EQ(tAdd.q.GetAngle(), tMul.q.GetAngle());
    EXPECT_EQ(tAdd.q.GetAngle(), tMul.q.GetAngle());
}  

TEST_F(ConfiguratorTest, SimVsCalc){
    Disturbance d(AVOID, b2Vec2(0.5, 0));
    Task task(d,LEFT);
    world_objects().emplace(world_objects().begin(), d.bodyFeatures());
    b2World world(b2Vec2(0, 0));
    Robot robot(&world);
    simResult result=task.bumping_that(world, 1, robot.body());
    float omega10Hz=task.getAction().getOmega(0.1); //angular vel/0.1s
    float theta=omega10Hz*result.step;
    EXPECT_NEAR(result.endPose.q.GetAngle(), theta, 0.00001);
    EXPECT_NEAR(approximate_angle(result.endPose.q.GetAngle(), LEFT, result.resultCode), M_PI_2, 0.00001);
    }


INSTANTIATE_TEST_CASE_P(Angles, LinearTransformationTest, testing::Values(0, M_PI_4, M_PI/6, M_PI_2, M_PI_2+M_PI/6,-M_PI_4, -M_PI/6, -M_PI_2, -M_PI_2-M_PI/6 ));