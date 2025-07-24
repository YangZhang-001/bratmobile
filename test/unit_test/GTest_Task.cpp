#include "test_classes.h"

class TaskTest: public Task, public ::testing::TestWithParam<Direction>{};

TEST_P(TaskTest, EndCriteriaAngleSign){
    direction=GetParam();
    action.init(direction);
    EXPECT_EQ(direction, GetParam());
    setEndCriteria();
    switch (direction){
        case LEFT:
            EXPECT_GT(endCriteria.angle.get_signed(), 0); break;
        case RIGHT:
            EXPECT_LT(endCriteria.angle.get_signed(), 0); break;
        default:
            EXPECT_FALSE(endCriteria.angle.isValid());break;
    }
}

INSTANTIATE_TEST_CASE_P(Directions, TaskTest, testing::Values(LEFT, RIGHT, DEFAULT));