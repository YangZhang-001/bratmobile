#include "test_classes.h"

class TaskTest: public Task, public ::testing::TestWithParam<Direction>{
    protected:
    void setDisturbance(const Disturbance& _dist){disturbance=_dist;};
    public:

    void TearDown(){
        taskInit(Disturbance(), DEFAULT);
    }

    EndCriteria taskInit(const Disturbance & _disturbance, Direction _direction){
        disturbance=_disturbance;
        direction=_direction;
        action.init(_direction);
        setEndCriteria();
        return endCriteria;
    }
    Task getTask(){
        return Task(disturbance, direction, b2Transform_zero, true);
    }
};

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

TEST_P(TaskTest, EndCriteriaAngleSignTarget){
    disturbance=Disturbance(PURSUE, b2Vec2(1.0,0));
    direction=GetParam();
    action.init(direction);
    EXPECT_EQ(direction, GetParam());
    setEndCriteria();
    switch (direction){
        case LEFT:
            EXPECT_EQ(endCriteria.angle.get_signed(), 0); break;
        case RIGHT:
            EXPECT_EQ(endCriteria.angle.get_signed(), 0); break;
        default:
            EXPECT_FALSE(endCriteria.angle.isValid());break;
    }
}

INSTANTIATE_TEST_CASE_P(Directions, TaskTest, testing::Values(LEFT, RIGHT, DEFAULT));


TEST_P(TaskTest, TaskHasFixedEndCriteria){
    Direction _direction=GetParam();    
    EndCriteria ec= taskInit(Disturbance(AVOID, b2Vec2(0.5,0), 0), _direction); //tracked d
    std::vector<BodyFeatures> objects({disturbance.bf});
    ClosedLoop_Tracker tracker;
    tracker.set_tracked_disturbance(disturbance);
    math::MulT(-(action.getTransform(LIDAR_SAMPLING_RATE)), disturbance);
    Pointf point(disturbance.getPosition().x, disturbance.getPosition().y);
    CoordinateContainer data2fp={point};
    tracker.track(*this, data2fp, objects);
    EXPECT_TRUE(ec==endCriteria);
}

TEST_F(TaskTest, AdjustEndCriteria){
    b2Transform transform;
    transform.q.Set(M_PI_4);
    endCriteria.angle.set(M_PI_2);
    endCriteria.angle.setValid(true);
    endCriteria.adjust(transform);
    EXPECT_LT(endCriteria.angle.get_signed(), M_PI_2);

}

class ConfiguratorTestTask:public DebugConfigurator, public TaskTest{};

TEST_P(ConfiguratorTestTask, AdjustSimTask){
    auto e=make_successful(MOVING_VERTEX);
    currentVertex=e.m_target;
    Direction _direction=GetParam();    
    EndCriteria ec= taskInit(Disturbance(AVOID, b2Vec2(0.5,0), 0), _direction); //tracked d
    transitionSystem[e.m_target].direction=GetParam();
    currentTask.set_direction(GetParam());
    transitionSystem[e.m_target].Di=disturbance;
    CLTrackerTest cltracker;
    register_tracker(&cltracker);
    cltracker.setDeltaTransform(-(action.getTransform(LIDAR_SAMPLING_RATE)));
    adjust_simulated_task(e.m_source, *this);
    EXPECT_TRUE(endCriteria.angle< ec.angle);
    EXPECT_TRUE(ec.distance ==endCriteria.distance);

}

INSTANTIATE_TEST_CASE_P(Directions, ConfiguratorTestTask, testing::Values(LEFT, RIGHT, DEFAULT));

TEST_P(TaskTest, TerminateEarly){
    float angle =0;
    if (GetParam()!=DEFAULT){
        angle=M_PI_4;
        if (GetParam()==RIGHT){
            angle=-angle;
        }
        endCriteria.angle.set(angle);
        endCriteria.angle.setValid(true);
    }
    direction=GetParam();
    action.init(direction);
    b2Transform bfPose;
    bfPose.p.x=.5;
    b2World world(GRAVITY);
    WorldBuilder wb;
    BodyFeatures bf(bfPose);
    disturbance=Disturbance(bf);
    affordance=disturbance.getAffIndex();
    EXPECT_EQ(disturbance.getAffIndex(), AVOID);
    disturbance.validate();
    bf.attention=true;
    wb.set_world_objects({bf});
    wb.buildWorld(world, start, direction);
    Robot robot(&world);
    simResult result=bumping_that(world, 1, robot.body);
    EXPECT_NEAR(robot.body->GetTransform().q.GetAngle(), angle, M_PI/(2*HZ));
}