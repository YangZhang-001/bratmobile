#include "../realWorldTestHeaders.h"
#include "stdio.h"
#include "test_classes.h"
#include <gtest/gtest.h>
class TestEnvironment;
const bool DEBUG = false;

/**
 * Tests for tracking unit tests
 */
class TestInputConfigurator : public UserInputConfigurator
{
    BodyFeatures initial_bf;

  public:
    friend class TestEnvironment;
    TestInputConfigurator () {}
    TestInputConfigurator (DirectionSetter *ds, AffordanceSetter *as)
        : UserInputConfigurator (ds, as)
    {
    }
    CoordinateContainer &getData2fp () { return data2fp; };
    const Disturbance &getDi () { return currentTask.get_disturbance (); }
    AffordanceIndex goalAffordance ()
    {
        return currentTask.get_disturbance ().getAffIndex ();
    }
    void set_world_objects (std::vector<BodyFeatures> bfs)
    {
        worldBuilder->set_world_objects (bfs);
    };

    Task &getGoal () { return controlGoal; }
    Task &getTask () { return currentTask; }
    void setRunning (bool r) { running = r; }

    void run ()
    {
        Spawner ();
        if (getIteration () > 1)
        {
            TrackingResult trackingResult (currentTask.get_disturbance ());
            trackingResult = tracker->track (
                (currentTask), data2fp, worldBuilder->get_world_objects ());
            update_graph (transitionSystem, trackingResult);
        }
        if (goal_changer != NULL)
        {
            if ((currentTask.is_over ()
                     && transitionSystem[currentVertex].direction != STOP
                 && m_plan.empty () && getIteration () > 1))
            {
                controlGoal = goal_changer->change_goal (controlGoal);
            }
        }
        change_task ();
        adjust_goal_expectation ();
        estimate_current_vertex ();
        printf ("current v=%li\n", currentVertex);
    }

    void MulPoints (b2Transform t)
    {
        CoordinateContainer data;
        for (auto p : data2fp)
        {
            b2Vec2 v = b2Mul (t, b2Vec2 (p.x, p.y));
            data.emplace (Pointf (v.x, v.y));
        }
        data2fp = data;
    }

    void cvMulPoints (b2Transform t)
    {
        cv::Mat affine = cv::getRotationMatrix2D (
            cv::Point2f (0, 0), double (t.q.GetAngle ()), double (1));
        // affine.at<double>(0, 2) = 0.4*t.q.s;
        // affine.at<double>(1, 2) = 0.4*t.q.c;
        std::vector<cv::Point2f> result;
        cv::transform (set2vec2f (data2fp), result, affine);
        data2fp.clear ();
        for (auto p : result)
        {
            data2fp.emplace (Pointf (p.x, p.y));
        }
    }
};

class TestInputConfiguratorFixture
    : public virtual TestInputConfigurator,
      public ::testing::TestWithParam<std::tuple<Direction, float> >
{
  public:
    TestInputConfiguratorFixture () {}

    Logger makeLogger ()
    {
        //std::string dumpFolder="benchmark", systemArchDir=dumpFolder+Logger::getSystemArchitecture();
        std::string testCaseDir = ::testing::UnitTest::GetInstance ()
                                      ->current_test_info ()
                                      ->name ();
        std::string valueParam = ::testing::UnitTest::GetInstance ()
                                     ->current_test_info ()
                                     ->value_param ();
        // if (testCaseDir[testCaseDir.size()-2]=='/'){
        //     testCaseDir.pop_back();
        //testCaseDir.pop_back();
        // }
        auto slash = testCaseDir.find_last_of ('/');
        testCaseDir.erase (slash, testCaseDir.size () - 1);
        if (valueParam[1] == '0')
        {
            valueParam = "/LEFT";
        }
        else if (valueParam[1] == '1')
        {
            valueParam = "/RIGHT";
        }
        return Logger (testCaseDir.c_str (), ".", valueParam.c_str (), false);
    }
};

class TestTracker : public ClosedLoop_Tracker
{

  public:
    b2PolygonShape getAttentionWindow () { return attention_window; }

    b2AABB getAttentionAABB ();

    Disturbance *get_tracked_disturbance () { return &tracked_disturbance; }

    void makeAttentionWindow (const Task &goal, const Task &currentTask)
    {
        ClosedLoop_Tracker::makeAttentionWindow (goal, currentTask);
    }
};

b2AABB TestTracker::getAttentionAABB ()
{
    b2AABB aabb;
    attention_window.ComputeAABB (&aabb, b2Transform_zero, 0);
    return aabb;
}

class TestEnvironment
    : public ::testing::TestWithParam<std::tuple<AffordanceIndex, Direction> >
{
  public:
    AffordanceSetter as = AffordanceSetter (NONE);
    DirectionSetter ds = DirectionSetter (UNDEFINED);
    AffordanceIndex affSolution = NONE; //what is the control goal?
    void SetUp () override
    {
        as = AffordanceSetter (std::get<0> (GetParam ()));
        ds = DirectionSetter (std::get<1> (GetParam ()));
        affSolution = std::get<0> (GetParam ());
    }

    BodyFeatures makeBF (TestInputConfigurator &configurator)
    {
        BodyFeatures bf;
        bf.halfLength = 0.05;
        bf.halfWidth = 0.01;
        float distance = .4; //40cm away
        if (std::get<0> (GetParam ()) == AVOID)
        {
            if (std::get<1> (GetParam ()) == DEFAULT)
            {
                bf.pose.p.y = distance;
            }
            else
            {
                bf.pose.p.x = distance;
            }
        }
        else if (std::get<0> (GetParam ()) == PURSUE)
        {
            if (std::get<1> (GetParam ()) == DEFAULT)
            {
                bf.pose.p.x = distance;
            }
            else if (std::get<1> (GetParam ()) == LEFT)
            {
                bf.pose.p.y = distance;
            }
            else if (std::get<1> (GetParam ()) == RIGHT)
            {
                bf.pose.p.y = -distance;
            }
        }
        configurator.set_world_objects (std::vector<BodyFeatures>{ bf });
        configurator.getData2fp ().emplace (Pointf (bf.pose.p.x, bf.pose.p.y));
        return bf;
    }

    void setConfiguratorBF (TestInputConfigurator &configurator,
                            BodyFeatures bf)
    {
        configurator.initial_bf = bf;
    }
};

TEST_P (TestEnvironment, AttentionWindow)
{
    TestTracker tracker;
    OneTaskController controller;
    TestInputConfigurator configurator (&ds, &as);
    BodyFeatures bf = makeBF (configurator);
    configurator.register_tracker (&tracker);
    configurator.register_controller (&controller);
    Disturbance goal (PURSUE, b2Vec2 (1, 0));
    configurator.init (Task (goal, UNDEFINED));
    configurator.Spawner (); //
    configurator.change_task ();
    configurator.adjust_goal_expectation ();
    configurator.estimate_current_vertex ();
    //tracker.on_new_reading(configurator.getGoal(), configurator.getTask());
    EXPECT_EQ (tracker.get_tracked_disturbance ()->pose ().p.x,
               configurator.getDi ().pose ().p.x);
    EXPECT_EQ (tracker.get_tracked_disturbance ()->pose ().p.y,
               configurator.getDi ().pose ().p.y);
    EXPECT_EQ (tracker.get_tracked_disturbance ()->pose ().q.GetAngle (),
               configurator.getDi ().pose ().q.GetAngle ());
    EXPECT_EQ (configurator.goalAffordance (), affSolution);
    if (std::get<0> (GetParam ()) == AVOID)
    {
        EXPECT_TRUE (overlaps (tracker.getAttentionWindow (),
                               tracker.get_tracked_disturbance ()));
    }
    EXPECT_EQ (configurator.getDi ().bf.pose.p.x, bf.pose.p.x);
    EXPECT_EQ (configurator.getDi ().bf.pose.p.y, bf.pose.p.y);
    //  }
}

TEST_P (TestEnvironment, Execution)
{
    OneTaskController controller;
    TestInputConfigurator configurator (&ds, &as);
    BodyFeatures bf = makeBF (configurator);
    TestTracker tracker;
    setConfiguratorBF (configurator, bf);
    configurator.register_tracker (&tracker);
    configurator.register_controller (&controller);
    Disturbance goal (PURSUE, b2Vec2 (1, 0));
    configurator.init (Task (goal, UNDEFINED));
    MotorInterface motor;
    configurator.registerInterface (&motor);
    int steps = 0;
    do
    {
        configurator.run ();
        b2Transform newPose = b2help::InvMul (
            configurator.getTask ().getAction ().getTransform (
                LIDAR_SAMPLING_RATE),
            bf.pose);
        configurator.set_data2fp ({ Pointf (newPose.p.x, newPose.p.y) });
        bf.pose = newPose;
        steps++;
        if (steps > 50)
            break;
    }
    while (!configurator.getTask ().is_over ());
    EXPECT_GT (steps, 1); //should take more than one step to complete task
    EXPECT_TRUE (configurator.getTask ().is_over ());
}

/**
 * @brief Tests how the system adapts to noise in task execution (e.g. if the turn is not perfectly 90 degrees)
 * 
 */
TEST_P (TestInputConfiguratorFixture, ExecutionNoise)
{
    // GTEST_SKIP();
    Logger logger = makeLogger ();
    TestTracker tracker;
    Wise_Controller wc;
    MotorInterface motor;
    control = &motor;
    register_tracker (&tracker);
    register_controller (&wc);
    init (DebugConfigurator::generateGoalTask ());
    data2fp = (CoordinateContainer{ Pointf (0.4, 0.01), Pointf (0.4, 0),
                                    Pointf (0.4, -0.01), Pointf (0.4, -0.02),
                                    Pointf (0.4, 0.02) });
    EXPECT_GT (data2fp.size (),
               1); //should take more than one step to complete task
    worldBuilder->set_world_objects (
        worldBuilder->getFeatures (data2fp, b2Transform_zero));
    EXPECT_GT (world_objects ().size (), 0);
    Disturbance obstacle (worldBuilder->get_world_objects ()[0]);
    obstacle.validate ();
    auto e1 = make_successful (MOVING_VERTEX,
                               std::get<0> (GetParam ())); //param 0 =direction
    float targetAngle (M_PI_2);
    transitionSystem[e1.m_target].Di = obstacle;
    if (std::get<0> (GetParam ()) == RIGHT)
    {
        targetAngle = -targetAngle;
    }
    transitionSystem[e1.m_target].endPose.q.Set (targetAngle);
    TrackingResult trackingResult (currentTask.get_disturbance ());
    float angleError = std::get<1> (GetParam ());
    b2Transform errorTransform
        = b2Transform (b2Vec2 (0, 0), b2Rot (DEG_TO_RAD_K * angleError)),
        deltaPose = errorTransform;
    set_plan ({ e1.m_target });
    int steps = -1;
    float desiredAngle = -M_PI;
    do
    {
        change_task ();
        if (steps < 0)
        {
            desiredAngle = currentTask.getEndCriteria ().angle.get_signed ();
        }
        adjust_goal_expectation ();
        estimate_current_vertex ();
        MulPoints (deltaPose);
        deltaPose
            = -currentTask.getAction ().getTransform (LIDAR_SAMPLING_RATE);
        worldBuilder->set_world_objects (
            worldBuilder->getFeatures (data2fp, b2Transform_zero));
        trackingResult = tracker.track ((currentTask), data2fp,
                                        worldBuilder->get_world_objects ());
        update_graph (transitionSystem, trackingResult);
        steps++;
        iteration++;
        if (steps > 50)
            break;
    }
    while (!currentTask.is_over ());
    b2Transform travelled_transform = tracker.getDeltaTransform ();
    //test value    //how far robot went            //desired angle                         //stop angle
    logger.log ("%f\t%f\t%f\t%f\t%f\t%i\n", angleError,
                travelled_transform.q.GetAngle (),
                currentTask.from_Di ().q.GetAngle (),
                b2Mul (errorTransform, travelled_transform).q.GetAngle (),
                desiredAngle, steps);
    logger.~Logger ();
    EXPECT_NEAR (fabs (tracker.getDeltaTransform ().q.GetAngle ()),
                 fabs (desiredAngle), 4.5 * DEG_TO_RAD_K);
    EXPECT_GT (fabs (tracker.getDeltaTransform ().q.GetAngle ()), 0);
    EXPECT_NEAR (currentTask.from_Di ().q.c, std::cos (-targetAngle),
                 std::cos (4.5 * DEG_TO_RAD_K));
    EXPECT_NEAR (currentTask.from_Di ().q.s, std::sin (-targetAngle),
                 std::sin (4.5 * DEG_TO_RAD_K));

    // EXPECT_GT(steps, 1); //should take more than one step to complete task
    // SUCCEED();
}

/**
 * @brief Test whether the current invmul method is accurate when turning left
 */
TEST_P (TestDeadReckoning, InvMulGoal)
{
    Task t = DebugConfigurator::generateGoalTask ();
    Task::Action a1, a2;
    auto directions = GetParam ();
    a1.init (std::get<0> (directions));
    a2.init (std::get<1> (directions));
    b2Transform t1 = (a1.getTransform (0.1)), t2 = a2.getTransform (0.1);
    for (int i = 0; i < 20; i++)
    {
        Configurator::InvMul (t1, t);
    }
    for (int i = 0; i < 20; i++)
    {
        Configurator::InvMul (t2, t);
    }
    float disturbanceAngle = t.get_disturbance ().pose ().q.GetAngle ();
    float t1Angle = t1.q.GetAngle ();
    if (std::get<0> (directions) == std::get<1> (directions))
    {
        EXPECT_EQ (std::signbit (disturbanceAngle), std::signbit (-t1Angle));
    }
    else
    {
        EXPECT_NEAR (disturbanceAngle, 0, M_PI / 10);
    }
}

INSTANTIATE_TEST_CASE_P (Directions, TestDeadReckoning,
                         ::testing::Combine (testing::Values (LEFT, RIGHT),
                                             testing::Values (LEFT, RIGHT)));

INSTANTIATE_TEST_CASE_P (Noise, TestInputConfiguratorFixture,
                         ::testing::Combine (testing::Values (LEFT, RIGHT),
                                             ::testing::Range (-91.0f, 91.0f,
                                                               1.0f)));

INSTANTIATE_TEST_CASE_P (
    Inputs, TestEnvironment,
    ::testing::Combine (
        ::testing::Values (PURSUE, AVOID),
        ::testing::Values (
            DEFAULT, LEFT,
            RIGHT))); //      a_ob  r_ob  a_go    r_go     Configurator::getGoalDisturbance()