#include "test_classes.h"
const bool DEBUG=false;

class TaskTest: public Task{
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

class TaskTestEndCriteria: public Task, public ::testing::TestWithParam<Direction>{
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

TEST_P(TaskTestEndCriteria, EndCriteriaAngleSign){
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

TEST_P(TaskTestEndCriteria, EndCriteriaAngleSignTarget){
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

INSTANTIATE_TEST_CASE_P(Directions, TaskTestEndCriteria, testing::Values(LEFT, RIGHT, DEFAULT));


TEST_P(TaskTestEndCriteria, TaskHasFixedEndCriteria){
    Direction _direction=GetParam();    
    EndCriteria ec= taskInit(Disturbance(AVOID, b2Vec2(0.5,0), 0), _direction); //tracked d
    std::vector<BodyFeatures> objects({disturbance.bf});
    CLTrackerTest tracker;
    tracker.set_tracked_disturbance(disturbance);
    math::MulT(-(action.getTransform(LIDAR_SAMPLING_RATE)), disturbance);
    Pointf point(disturbance.getPosition().x, disturbance.getPosition().y);
    CoordinateContainer data2fp={point};
    tracker.track(*this, data2fp, objects);
    EXPECT_TRUE(ec==endCriteria);
}

TEST_F(TaskTestEndCriteria, AdjustEndCriteria){
    b2Transform transform;
    transform.q.Set(M_PI_4);
    endCriteria.angle.set(M_PI_2);
    endCriteria.angle.setValid(true);
    endCriteria.adjust(transform);
    EXPECT_LT(endCriteria.angle.get_signed(), M_PI_2);
}

class ConfiguratorTestTask:public DebugConfigurator, public TaskTestEndCriteria{};

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
    cltracker.setDeltaTransform((action.getTransform(LIDAR_SAMPLING_RATE)));
    adjust_simulated_task(e.m_source, *this);
    EXPECT_TRUE(endCriteria.angle< ec.angle);
    if (GetParam()!=DEFAULT){
        EXPECT_EQ(endCriteria.angle.get(), ec.angle.get()-fabs(cltracker.getDeltaTransform().q.GetAngle()));
    }
    EXPECT_TRUE(ec.distance ==endCriteria.distance);

}

TEST_P(ConfiguratorTestTask, AdjustSimOppositeTask){
    auto e=make_successful(MOVING_VERTEX);
    currentVertex=e.m_target;
    Direction _direction=getOppositeDirection(GetParam()).second;     
    EndCriteria ec= taskInit(Disturbance(AVOID, b2Vec2(0.5,0), 0), _direction); //tracked d
    transitionSystem[e.m_target].direction=GetParam();
    currentTask.set_direction(GetParam());
    transitionSystem[e.m_target].Di=disturbance;
    CLTrackerTest cltracker;
    register_tracker(&cltracker);
    cltracker.setDeltaTransform((action.getTransform(LIDAR_SAMPLING_RATE)));
    adjust_simulated_task(e.m_source, *this);
    EXPECT_TRUE(ec.angle<endCriteria.angle);
    if (GetParam()!=DEFAULT){
        EXPECT_EQ(endCriteria.angle.get(), ec.angle.get()+fabs(cltracker.getDeltaTransform().q.GetAngle()));
    }
    EXPECT_TRUE(ec.distance ==endCriteria.distance);
}

TEST_P(ConfiguratorTestTask, addTurnToPriorityQueue){
    auto e=make_successful(MOVING_VERTEX);
    currentVertex=e.m_target;
    vertex_set_direction(currentVertex, GetParam());
    std::vector<vertexDescriptor>evaluationQ={currentVertex}, priorityQ;
    std::set<vertexDescriptor>closed;
    backtrack(evaluationQ, priorityQ, closed, m_plan);
    EXPECT_GT(priorityQ.size(), 0);
}



INSTANTIATE_TEST_CASE_P(Directions, ConfiguratorTestTask, testing::Values(LEFT, RIGHT, DEFAULT));

class TaskTestTermination: public TaskTest, public testing::TestWithParam<std::tuple<Direction, float>>{};

TEST_P(TaskTestTermination, TerminateEarly){
    float angle =0;
    if (std::get<0>(GetParam())!=DEFAULT){
        angle=std::get<1>(GetParam());
        if (std::get<0>(GetParam())==RIGHT){
            angle=-angle;
        }
        endCriteria.angle.set(angle);
        endCriteria.angle.setValid(true);
    }
    direction=std::get<0>(GetParam());
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
    simResult result=bumping_that(world, 1, robot.body());
    EXPECT_NEAR(robot.body()->GetTransform().q.GetAngle(), angle, M_PI/(2*HZ));
}

/**
 * @brief checking if an empty goal would return finished if plan interrupted midway
 */
TEST_F(TaskTestEndCriteria, CheckEndMidWay){
    start.p.x=-.00;
    b2Transform end(b2Vec2(0.979999, 0), b2Rot(0));
    bool result=checkEnded(end, UNDEFINED, true).ended;
    EXPECT_FALSE(result); //if doesnt reach box2d range it returns false
}

INSTANTIATE_TEST_CASE_P(TerminateEarly, TaskTestTermination, ::testing::Combine(testing::Values(LEFT, RIGHT, DEFAULT), ::testing::Values(M_PI_4)));
INSTANTIATE_TEST_CASE_P(TerminateLate, TaskTestTermination, ::testing::Combine(testing::Values(LEFT, RIGHT, DEFAULT), ::testing::Values(M_PI_4+M_PI_2)));

