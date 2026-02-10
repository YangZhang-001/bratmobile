#include "test_classes.h"
#include <gtest/gtest.h>
const bool DEBUG=false;


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
    EXPECT_EQ(currentVertex, MOVING_VERTEX);
}


TEST_F(ConfiguratorTest, DummyVertex){
    init();
    dummy_vertex(MOVING_VERTEX); 
    EXPECT_TRUE(currentTask.is_over());
    EXPECT_EQ(boost::out_degree(MOVING_VERTEX, transitionSystem), 1);
    EXPECT_FALSE(boost::edge(MOVING_VERTEX, MOVING_VERTEX, transitionSystem).second);
}

TEST_F(ConfiguratorTest, VisitingTS){
    iteration=1;
    make_module();
    auto es=boost::edges(transitionSystem);
    for (auto ei=es.first; ei!=es.second; ei++){
        EXPECT_EQ(transitionSystem[*ei].it_observed,iteration);
    }
    VisitedEdge ve(&transitionSystem, iteration+1);
    EXPECT_EQ(ve.getIteration(), 2);
    iteration++;
    EXPECT_EQ(n_visitedEdges(), 0);
}

TEST_F(ConfiguratorTest, PreExplore){
    init();
    iteration++;
    pre_explore();
}

TEST_F(ConfiguratorTest, VisitedTS){
    iteration=1;
    make_module();
    auto es=boost::edges(transitionSystem);
    for (auto ei=es.first; ei!=es.second; ei++){
        EXPECT_EQ(transitionSystem[*ei].it_observed,iteration);
    }
    iteration=2;
    auto e=make_successful();
    VisitedEdge ve(&transitionSystem, iteration);
    EXPECT_EQ(ve.getIteration(), 2);
    EXPECT_EQ(n_visitedEdges(), 1);
}

TEST_F(ConfiguratorTest, VisitingEdge){
    iteration=1;
    make_module();
    auto es=boost::edges(transitionSystem);
    for (auto ei=es.first; ei!=es.second; ei++){
        EXPECT_EQ(transitionSystem[*ei].it_observed,iteration);
    }
    VisitedEdge ve(&transitionSystem, iteration+1);
    EXPECT_EQ(ve.getIteration(), 2);
    auto e=boost::edge(2, 3, transitionSystem);
    EXPECT_FALSE(ve(e.first));
}

TEST_F(ConfiguratorTest, CurrentVertices){
    init();
    EXPECT_EQ(current_vertices.size(), 0);
}

TEST_F(ConfiguratorTest, CurrentVerticesDummy){
    init();
    dummy_vertex(MOVING_VERTEX);
    EXPECT_EQ(current_vertices.size(), 1);
}


TEST(VisitedEdge, Return){
    TransitionSystem ts(2);
    auto e=boost::add_edge(0, 1, ts);
    ts[e.first].it_observed=1;
    VisitedEdge ve(&ts, 2);
    EXPECT_FALSE(ve(e.first));
}

TEST_P(ConfiguratorTestGetGoal, GetDisturbanceGoal){
    Disturbance solution=controlGoal.get_disturbance();
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(0.55, 0, 0, 0.02, 0.05);
    vertex_setup(currentVertex,Disturbance(bf));
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, std::get<2>(GetParam()), transitionSystem[currentVertex].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, solution.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, solution.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), solution.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, solution.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, solution.bf.halfWidth);
}

INSTANTIATE_TEST_CASE_P(DisturbanceIsGoal, ConfiguratorTestGetGoal, ::testing::Values(
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), DEFAULT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), DEFAULT, RIGHT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), LEFT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), RIGHT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, RIGHT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, LEFT)));


TEST_P(ConfiguratorTestGetObstacle, GetDisturbanceObstacle){
    EXPECT_EQ(transitionSystem.m_vertices.size(),2);
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    Disturbance solution(bf);
    vertex_setup(currentVertex, solution);
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, std::get<2>(GetParam()), transitionSystem[currentVertex].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, solution.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, solution.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), solution.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, solution.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, solution.bf.halfWidth);
}

INSTANTIATE_TEST_CASE_P(DisturbanceIsObstacle, ConfiguratorTestGetObstacle, ::testing::Values(
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.4, 0.0), b2Rot(M_PI_2)), LEFT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.4, 0.0), b2Rot(M_PI_2)), RIGHT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.40, 0.31), b2Rot(-M_PI_2)), RIGHT, DEFAULT)));

/**
 * @brief Crash on the way to goal
 * 
 */
TEST_F(ConfiguratorTestGetObstacle, CrashToGoal){
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    transitionSystem[currentVertex].Di=Disturbance(PURSUE, b2Vec2(1.0, 0)); //current task was avoiding
    transitionSystem[currentVertex].Dn=Disturbance(bf); //current task was avoiding
    transitionSystem[currentVertex].Dn.validate();
    Disturbance solution=transitionSystem[currentVertex].Dn;
    transitionSystem[currentVertex].direction=DEFAULT;
    transitionSystem[currentVertex].endPose.p.x=0.4;
    vertex_options_push_back(currentVertex, LEFT);
    Disturbance Di= getDisturbance(transitionSystem, currentVertex, world, LEFT, transitionSystem[currentVertex].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, solution.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, solution.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), solution.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, solution.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, solution.bf.halfWidth);
}

TEST_F(ConfiguratorTestGetObstacle, AvoidNoGoal){
    init(Task());
    EXPECT_FALSE(controlGoal.get_disturbance().isValid());
    EXPECT_EQ(controlGoal.get_disturbance().getAffIndex(), NONE);
    b2World world(GRAVITY);
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    bf.attention=1;
    transitionSystem[MOVING_VERTEX].Di=Disturbance(bf); //current task was avoiding
    transitionSystem[MOVING_VERTEX].Di.validate();
    Disturbance solution=transitionSystem[MOVING_VERTEX].Di;
    transitionSystem[MOVING_VERTEX].direction=STOP;
    vertex_options_push_back(MOVING_VERTEX, LEFT);
    Disturbance Di= getDisturbance(transitionSystem, MOVING_VERTEX, world, LEFT, transitionSystem[MOVING_VERTEX].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, solution.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, solution.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), solution.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, solution.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, solution.bf.halfWidth);
}

TEST_F(ConfiguratorTest, GetDisturbance180Turn){
    init(generateGoalTask());
    b2World world(GRAVITY);
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05);
    bf.attention=1;
    auto v1=boost::add_vertex(transitionSystem);
    boost::add_edge(currentVertex, v1, transitionSystem);
    vertex_set_direction(v1, LEFT);
    transitionSystem[v1].Di=Disturbance(bf); //current task was avoiding
    transitionSystem[v1].Di.validate();
    vertex_options_push_back(v1, LEFT);
    Disturbance Di= getDisturbance(transitionSystem, v1, world, LEFT, transitionSystem[v1].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, bf.halfWidth);

}

TEST_F(ConfiguratorTest, UpdateGraph){
    Disturbance Di(PURSUE, b2Vec2(0.81, 0.23)), Dn(AVOID, b2Vec2(0.22, 0)), goal(PURSUE, b2Vec2(1.0, 0));
    init(Task(goal, UNDEFINED));
    dummy_vertex(MOVING_VERTEX);
    transitionSystem[1].Di=Di;
    transitionSystem[1].Dn=Dn;
    transitionSystem[1].direction=DEFAULT;
    //currentTask=Task(Dn, DEFAULT, b2Transform_zero, true);
    TrackingResult tr(Dn);
    tr.displacement=b2Transform(b2Vec2(.5, .27), b2Rot(M_PI_4));
    update_graph(transitionSystem, tr);
    EXPECT_FALSE(transitionSystem[1].Di.pose()==Di.pose());
    EXPECT_FALSE(transitionSystem[1].Dn.pose()==Dn.pose());
    EXPECT_FALSE(controlGoal.get_disturbance().pose()==goal.pose());
    EXPECT_TRUE(currentTask.get_disturbance().pose()==Dn.pose());
    EXPECT_NE(controlGoal.getStart().p.Length(), 0);
}


TEST(GraphTools, ToTaskEnd){
    TransitionSystem transitionSystem(5);
    auto e0=boost::add_edge(0, 1, transitionSystem);
    auto e1=boost::add_edge(1, 2, transitionSystem);
    auto e2=boost::add_edge(0, 3, transitionSystem);
    auto e3=boost::add_edge(2, 4, transitionSystem);
    transitionSystem[0].direction=DEFAULT;
    transitionSystem[1].direction=DEFAULT;
    transitionSystem[2].direction=DEFAULT;
    transitionSystem[4].direction=LEFT;
    transitionSystem[3].direction=RIGHT;
    std::vector <vertexDescriptor>plan={0, 1, 2, 4};
    std::vector <vertexDescriptor>::iterator it=plan.begin(), result=plan.end();
    edgeDescriptor e=e0.first;
    result=gt::to_task_end(e, transitionSystem, plan, it);
    EXPECT_EQ(e, e3.first);
    EXPECT_EQ(*result, 2);
    EXPECT_TRUE(transitionSystem[e.m_target].direction==LEFT);
    
}

TEST_P(ConfiguratorTestTransitionMatrix, naive){
    EXPECT_EQ(n_vertices(), 1);
    Disturbance target;
    if (std::get<0>(GetParam())!=b2Transform_inf){
        target=Disturbance(PURSUE, std::get<0>(GetParam()).p, std::get<0>(GetParam()).q.GetAngle());
        target.validate();
        Task goal(target, DEFAULT);
        init(goal);
    }
    dummy_vertex(MOVING_VERTEX);
    edgeDescriptor e= make_successful(1);
    transitionSystem[e].step=1;
    transitionSystem[e.m_target].direction=std::get<1>(GetParam());
    transitionSystem[e.m_target].outcome=std::get<2>(GetParam());
    applyTransitionMatrix(e.m_target, std::get<1>(GetParam()), false, currentVertex, m_plan);
    int expected=expectedOptions(std::get<1>(GetParam()), std::get<2>(GetParam()), target);
    EXPECT_EQ(transitionSystem[e.m_target].options.size(), expected);
}


/**
 * @brief Simulates finding option for task executing
 * 
 */
TEST_P(ConfiguratorTestTransitionMatrix, InPlanNotVisited0){
    EXPECT_EQ(n_vertices(), 1); 
    currentTask.set_direction(std::get<1>(GetParam()));
    vertex_set_direction(currentVertex, std::get<1>(GetParam()));
    if (std::get<1>(GetParam())==UNDEFINED || std::get<1>(GetParam())==STOP){
        return;
    }
    //dummy_vertex(MOVING_VERTEX);
    currentVertex = boost::add_vertex(transitionSystem);
    make_module(currentVertex);
    EXPECT_EQ(n_vertices(), 7); 
    planIsDirection(std::get<1>(GetParam()));
    currentVertex=MOVING_VERTEX;
    current_vertices={2};
    currentTask.set_change(false);
    currentTask.setMotorStep(20);
    iteration=2;
    applyTransitionMatrix(MOVING_VERTEX, currentTask.get_direction(), false, MOVING_VERTEX, m_plan);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 1);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options[0], currentTask.get_direction());
}
TEST_P(ConfiguratorTestTransitionMatrix, InPlanNotVisitedCV){
    EXPECT_EQ(n_vertices(), 1); 
    currentTask.set_direction(std::get<1>(GetParam()));
    vertex_set_direction(currentVertex, std::get<1>(GetParam()));
    if (std::get<1>(GetParam())==UNDEFINED || std::get<1>(GetParam())==STOP){
        return;
    }
    //dummy_vertex(MOVING_VERTEX);
    currentVertex = boost::add_vertex(transitionSystem);
    make_module(currentVertex);
    EXPECT_EQ(n_vertices(), 7); 
    planIsDirection(std::get<1>(GetParam()));
    currentVertex=2;
    current_vertices={2};
    currentTask.set_change(false);
    currentTask.setMotorStep(20);
    iteration=2;
    applyTransitionMatrix(MOVING_VERTEX, currentTask.get_direction(), false, MOVING_VERTEX, m_plan);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 1);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options[0], currentTask.get_direction());
}

TEST_P(ConfiguratorTestTransitionMatrix, InPlanVisited){
    Direction direction=std::get<1>(GetParam());
    currentTask.set_direction(direction);
    if (direction==UNDEFINED || direction==STOP){
        return;
    }
    setAllVisited(); //just movingvertex
    currentVertex = boost::add_vertex(transitionSystem);
    current_vertices={currentVertex};
    make_module(currentVertex);
    EXPECT_EQ(n_vertices(), 7); 
    planIsDirection(direction);
    iteration=2;
    edgeDescriptor e=edgeDescriptor();
    std::vector<Direction> solution={DEFAULT, LEFT, RIGHT};
    simResult::resultType outcome=std::get<2>(GetParam());
    switch(outcome){
        case simResult::crashed:
            e= make_v1_crashed(MOVING_VERTEX);
            erase_from_vector(solution, std::get<1>(GetParam()));
            break;
        default:
            e=make_successful(MOVING_VERTEX);
            transitionSystem[e.m_target].outcome=std::get<2>(GetParam());
            solution.clear();
            break;
    }
    transitionSystem[e.m_target].direction=transitionSystem[m_plan[0]].direction;
    setPhi(transitionSystem[e.m_target]);
    applyTransitionMatrix(MOVING_VERTEX, transitionSystem[e.m_target].direction, false, MOVING_VERTEX, m_plan);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options, solution);
}



INSTANTIATE_TEST_CASE_P(SimulationOutcomes, ConfiguratorTestTransitionMatrix, ::testing::Values(
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), DEFAULT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), LEFT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), UNDEFINED, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), DEFAULT, simResult::crashed),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), DEFAULT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), DEFAULT, simResult::safeForNow),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(-1.0,0), b2Rot(-M_PI)), RIGHT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform_inf, DEFAULT, simResult::crashed),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform_inf, DEFAULT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform_inf, DEFAULT, simResult::safeForNow),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), STOP, simResult::crashed),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), STOP, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(1.0,0), b2Rot(0)), STOP, simResult::safeForNow),


                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), DEFAULT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), LEFT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), UNDEFINED, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), DEFAULT, simResult::crashed),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), DEFAULT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), DEFAULT, simResult::safeForNow),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(-M_PI)), RIGHT, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), STOP, simResult::crashed),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), STOP, simResult::successful),
                                                                                std::tuple<b2Transform, Direction, simResult::resultType>(b2Transform(b2Vec2(.78,.2), b2Rot(0)), STOP, simResult::safeForNow)
));



/**
 * @brief Just checking it doens't return garbage
 * 
 */
TEST(AffineTransform, NotReturnGarbage){
    b2Vec2 position(1.0,0);
    b2Transform shift(position, b2Rot(0));
    Disturbance d(PURSUE, position);
    Task task(d, DEFAULT);
    Configurator::MulT(-shift, task);
    EXPECT_EQ(task.get_disturbance().getPosition().x, 2);
    EXPECT_EQ(task.get_disturbance().getPosition().y, 0);
    EXPECT_EQ(task.get_disturbance().pose().q.GetAngle(), 0);
    EXPECT_EQ(task.getStart(), shift);
}


/**
 * @brief Tests whether the tracked disturbance presents an unexpected shitf, the robot is able to track the goal assuming constant relationships between obstacle and goal
 * 
 */
TEST_P(ConfiguratorTest2DT, adjustGoal){
    /**
     * @brief Setup: robot drives towards an obstacle and has a target position behind it
     * the obstacle shifts unpredictably and the robot has to adjust the expectation of the goal,
     * assuming that the ratio between obstacle and goal is constant
     * 
     */
    Task goal(Disturbance(PURSUE, b2Vec2(1.0, 0)), DEFAULT);
    Disturbance obstacle(PURSUE, b2Vec2(0.45, 0), 0); //robot is driving towards an obstacle before it avoids it
    init(goal);
    currentTask=Task(obstacle, DEFAULT, b2Transform_zero,true);
    TrackingResult tr(currentTask.get_disturbance());
    tr.displacement=GetParam();
    ConfiguratorTest::Manual_WiseController controller;
    register_controller(&controller);
    controller.set_disturbance(obstacle);
    controller.set_Di_to_goal(goal.get_disturbance());
    vertexDescriptor v1;
    vertex_set_options(0, std::vector<Direction>(DEFAULT));
    add_vertex_now(MOVING_VERTEX, v1, goal.get_disturbance());
    m_plan={v1};
    math::MulT(tr.displacement, tr.displacement);
    update_graph(transitionSystem, tr);
    b2Transform expected =b2MulT(controller.get_disturbance().pose(), transitionSystem[plan_end()].Di.pose()); //position of goal wrt current disturbance
    /**/
    adjust_goal_expectation(); //what we're actually testing
    b2Transform observed =b2MulT(currentTask.get_disturbance().pose(), controlGoal.get_disturbance().pose());
    b2Transform difference=expected-observed;
    EXPECT_LT(difference.p.Length(),0.001);
    EXPECT_LT(fabs(difference.q.GetAngle()),0.001);

}

INSTANTIATE_TEST_CASE_P(AdjustGoal, 
                        ConfiguratorTest2DT, 
                        ::testing::Values(b2Transform(b2Vec2(0, 0), b2Rot(0)),
                                          b2Transform(b2Vec2(0, 0), b2Rot(M_PI_2)), 
                                          b2Transform(b2Vec2(0, 0), b2Rot(-0.23)),
                                          b2Transform(b2Vec2(0.4, 0), b2Rot(0)),
                                          b2Transform(b2Vec2(0, 0.3), b2Rot(0)),
                                          b2Transform(b2Vec2(-0.4, 0.3), b2Rot(0)),
                                          b2Transform(b2Vec2(0, -0.3), b2Rot(0.12)),
                                          b2Transform(b2Vec2(-0.4, 0), b2Rot(0.12)),
                                          b2Transform(b2Vec2(0.8, -0.05), b2Rot(-0.45))));

TEST_P(ConfiguratorTest32DT, splitTask){
   // b2Transform start=std::get<0>(GetParam());
    dummy_vertex(MOVING_VERTEX);
    transitionSystem[currentVertex].start=std::get<0>(GetParam());
    transitionSystem[currentVertex].endPose=std::get<0>(GetParam());
    vertexDescriptor v1=make_v1_crashed(currentVertex, std::get<0>(GetParam()), std::get<1>(GetParam()), std::get<2>(GetParam())).m_target;
    std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, currentVertex);
    b2Vec2 endPosition=std::get<1>(GetParam()).p;
    int expected_splitSize=std::min(int(endPosition.Length()/(simulationStep+0.00001))+2, 3);
    
    EXPECT_EQ(split.size(), expected_splitSize);
    int ct=1;
    for (vertexDescriptor v:split){
        float step_size=(transitionSystem[v].endPose.p-transitionSystem[v].start.p).Length();
        //EXPECT_GE(step_size, simulationStep+0.00001);
        EXPECT_FALSE(transitionSystem[v].Di.isValid());
        if(ct<(split.size())){
            EXPECT_EQ(transitionSystem[v].outcome, simResult::safeForNow);
        }
        if (ct==(split.size())){
            EXPECT_EQ(transitionSystem[v].outcome, simResult::crashed);
        }
        ct++;
    }
    
}

TEST_F(ConfiguratorTest, splitWithoutDummy){
    ClosedLoop_Tracker tracker;
    register_tracker(&tracker);
    dummy_vertex(MOVING_VERTEX);
    make_module(currentVertex);
    vertexDescriptor v1=n_vertices()-1, v0=n_vertices()-2;
    vertex_set_outcome(v1, simResult::crashed);
    std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, v0);
    EXPECT_EQ(split.size(), 2);
}



INSTANTIATE_TEST_CASE_P(SplitTask, ConfiguratorTest32DT, ::testing::Values(std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.6, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0, 0), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.26, -0.01), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.265, -0.16), b2Rot(0))),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0, 0), b2Rot(M_PI_2)), b2Transform(b2Vec2(0, 0.6), b2Rot(M_PI_2)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0.2, 0.27), b2Rot(0)), b2Transform(b2Vec2(0.75, 0.27), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0.27, 0.2), b2Rot(M_PI_2)), b2Transform(b2Vec2(.27, 0.75), b2Rot(M_PI_2)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.27, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.18, 0), b2Rot(0)), b2Transform_zero)) 
                                                                   );



TEST_P(ConfiguratorBacktrackTest, Backtrack){
        //make three branches (all default, one crashed the other two successful)
        auto v1=make_v1_crashed().m_target;
        auto v2=make_successful().m_target;
        auto v3=make_successful().m_target;
        setAllVisited();
        //make teh other two vertices rotations(just to give them a lower phi value)
        transitionSystem[v2].endPose.q.Set(M_PI_2);
        transitionSystem[v3].endPose.q.Set(-M_PI_2);
        std::vector <vertexDescriptor> evaluationQ={v1, v2, v3}, all=evaluationQ, priorityQ;
        std::set <vertexDescriptor> closed;
        //backtrack
        backtrack(evaluationQ, priorityQ, closed, m_plan);
        for (vertexDescriptor v:all){
            EXPECT_TRUE(transitionSystem[v].visited()); //check vertex marked as visited
        }
        EXPECT_NE(priorityQ[0], v1); //check that v1 is not the vertex with highest priority
}

TEST_F(ConfiguratorTest, SkipClosedBT){
    dummy_vertex(MOVING_VERTEX);
    vertexDescriptor v1=boost::add_vertex(transitionSystem);
    boost::add_edge(currentVertex, v1, transitionSystem);
    transitionSystem[v1].outcome=simResult::successful;
    std::vector <vertexDescriptor> evaluationQ={v1}, priorityQ;
    std::set <vertexDescriptor> closed={v1};
    backtrack(evaluationQ, priorityQ, closed, m_plan);
    EXPECT_TRUE(priorityQ.empty());
    EXPECT_TRUE(priorityQ.size()==0);
}

TEST_F(ConfiguratorTest, SkipClosedPQ){
    dummy_vertex(MOVING_VERTEX);
    vertexDescriptor v1=boost::add_vertex(transitionSystem);
    boost::add_edge(currentVertex, v1, transitionSystem);
    transitionSystem[v1].outcome=simResult::successful;
    std::vector <vertexDescriptor> evaluationQ={v1}, priorityQ;
    std::set <vertexDescriptor> closed={v1};
    addToPriorityQueue(v1, priorityQ, closed);
    EXPECT_TRUE(priorityQ.empty());
    EXPECT_TRUE(priorityQ.size()==0);
}

TEST_P(ConfiguratorTakeBool, AddToClosedSet){
    bool fullEdges=GetParam();
    iteration=1;
    shallowExpand(currentVertex);
    std::set<vertexDescriptor> closed;
    EXPECT_EQ(closeVertex(closed, currentVertex), fullEdges);
    EXPECT_EQ(closed.size(), fullEdges);
}

TEST_P(ConfiguratorTakeBool, GetExploredTransitions){
    int solution=1;
    if (GetParam()){
        solution=3;
    }
    iteration=1;
    shallowExpand(currentVertex);
    std::vector <Direction> directions={DEFAULT,LEFT, RIGHT};
    EXPECT_EQ(getExploredDirections(currentVertex, directions).size(), solution);

}

TEST_P(ConfiguratorTakeBool, startRecycle){
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4}, add;
    std::vector<std::vector<vertexDescriptor>> paths;
    paths.emplace_back(std::vector<vertexDescriptor>({14, 1, 3, 4}));
    vertexDescriptor solution=DUMMY;
    make_ts(avoid, desiredPlan, true); 
    currentVertex=*desiredPlan.rbegin();
    if (!GetParam()){
        transitionSystem[2].outcome=simResult::successful;
        solution=currentVertex;
    }
    EXPECT_EQ(getRecyclingStart(currentVertex,2, 2), solution);
    
}

TEST_P(ConfiguratorTakeBool, FrontierVertices){
    make_module(MOVING_VERTEX);
    int solution=3;
    if (GetParam()){
        transitionSystem[1].outcome=simResult::crashed;
    }
    else{
        transitionSystem[1].outcome=simResult::successful;
    }
    ExecutionInfo info=package_info();
    auto vs=boost::vertices(transitionSystem);
    for (auto vi=vs.first; vi!=vs.second; ++vi){
        transitionSystem[*vi].phi=evaluationFunction(estimateCost(transitionSystem[*vi], b2Transform_zero, transitionSystem[*vi].direction, controlGoal), *vi, m_plan);
    }
    std::vector<Frontier> frontiers=frontierVertices(MOVING_VERTEX, transitionSystem, info);
    EXPECT_EQ(frontiers.size(), solution);
}

TEST_F(ConfiguratorTest, IncompleteFrontier){
    auto es=make_successful(MOVING_VERTEX);
    int solution=1;
    auto ec =make_v1_crashed(MOVING_VERTEX);
    transitionSystem[ec.m_target].direction=LEFT;
    ExecutionInfo info=package_info();
    auto vs=boost::vertices(transitionSystem);
    for (auto vi=vs.first; vi!=vs.second; ++vi){
        transitionSystem[*vi].phi=evaluationFunction(estimateCost(transitionSystem[*vi], b2Transform_zero, transitionSystem[*vi].direction, controlGoal), *vi, m_plan);
    }
    std::vector<Frontier> frontiers=frontierVertices(MOVING_VERTEX, transitionSystem, info);
    EXPECT_EQ(frontiers.size(), solution);
}

TEST_P(ConfiguratorTakeBool, PropagateDisturbance){
    dummy_vertex(MOVING_VERTEX);
    auto e=make_v1_crashed(currentVertex);
    bool hasSameDn=true;
    if (GetParam()){
        vertex_set_direction(currentVertex, transitionSystem[e.m_target].direction);
        hasSameDn=false;
    }
    propagateD(e.m_target, currentVertex);
    EXPECT_EQ(vertex_get_Dn(e.m_target)==vertex_get_Dn(currentVertex), hasSameDn);
}

TEST_F(ConfiguratorTest, CorrectQueue){
    transitionSystem=TransitionSystem(5);
    std::vector<vertexDescriptor>pq={2, 3, 4}, plan={1, 2}, solution={2, 3, 1};
    vertexDescriptor v=4, startRecycle=1;
    correctQueue(pq, v, startRecycle, plan.size());
    EXPECT_EQ(pq, solution);
}

TEST_P(ConfiguratorTakeBool, CheckVectorForPredicate){
    vertexDescriptor v1;
    Edge e;
    bool solution=false; 
    if (GetParam()){
        iteration=2;
        solution=true;
    }
    transitionSystem[currentVertex].options={DEFAULT, LEFT, RIGHT};
    add_vertex_now(currentVertex, v1, controlGoal.get_disturbance(), e,true);
    SameIteration si(transitionSystem, 2);
    std::vector<edgeDescriptor> ie=inEdges(v1, UNDEFINED);
    auto it=check_vector_for(ie, si);
    bool hasResult=it!=ie.end();
    EXPECT_EQ(hasResult, solution);
}

/**
 * @brief Testing that evaluation function is not affected by direction
 * 
 */
TEST_P(ConfiguratorTakeBool, EvaluationFunctionSame){
    std::vector <Direction> directions={LEFT, RIGHT, DEFAULT};
    for (Direction d: directions){
        dummy_vertex(MOVING_VERTEX);
        vertex_set_direction(currentVertex, d);
        if (GetParam()){
            transitionSystem[currentVertex].outcome=simResult::successful;
        }
        else{
            transitionSystem[currentVertex].outcome=simResult::crashed;
            transitionSystem[currentVertex].Dn.isValid();
            transitionSystem[currentVertex].Dn.setPose(b2Transform(b2Vec2(0.5, 0), b2Rot(0)));
        }
        EndedResult er= estimateCost(transitionSystem[currentVertex], b2Transform_zero, d, controlGoal);
        vertex_set_phi(currentVertex, evaluationFunction(er, currentVertex, m_plan));
    }
    EXPECT_EQ(transitionSystem[1].phi, transitionSystem[3].phi);
    EXPECT_EQ(transitionSystem[2].phi, transitionSystem[3].phi);
}

TEST_P(ConfiguratorTakeBool, EvaluationFunctionTurn){
    std::vector <Direction> directions={LEFT, RIGHT, DEFAULT};
    for (Direction d: directions){
        dummy_vertex(MOVING_VERTEX);
        vertex_set_direction(currentVertex, d);
        if(d==LEFT){
            transitionSystem[currentVertex].endPose.q.Set(M_PI_2);
        }
        else if (d==RIGHT){
            transitionSystem[currentVertex].endPose.q.Set(-M_PI_2);
        }
        if (GetParam()){
            transitionSystem[currentVertex].outcome=simResult::successful;
        }
        else{
            transitionSystem[currentVertex].outcome=simResult::crashed;
            transitionSystem[currentVertex].Dn.isValid();
            transitionSystem[currentVertex].Dn.setPose(b2Transform(b2Vec2(0.5, 0), b2Rot(0)));
        }
        EndedResult er= estimateCost(transitionSystem[currentVertex], b2Transform_zero, d, controlGoal);
        vertex_set_phi(currentVertex, evaluationFunction(er, currentVertex, m_plan));
    }
    EXPECT_EQ(transitionSystem[1].phi, transitionSystem[3].phi);
    EXPECT_EQ(transitionSystem[2].phi, transitionSystem[3].phi);
}


INSTANTIATE_TEST_CASE_P(Bool, ConfiguratorTakeBool, testing::Bool());

INSTANTIATE_TEST_CASE_P(Backtrack, ConfiguratorBacktrackTest, ::testing::Values(std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.6, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0, 0), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.26, -0.01), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.265, -0.16), b2Rot(0)))));


class ConfiguratorTaskVerticesTest: public ConfiguratorTest, public testing::WithParamInterface<std::tuple<bool, Direction>>{};

TEST_P(ConfiguratorTaskVerticesTest, SameTaskVertices){
    std::cout<<"Testing if AttentiveConfigurator::task_vertices returns vertices of all the same task"<<std::endl;
    Direction direction=std::get<1>(GetParam());
    auto e1=make_successful(MOVING_VERTEX);
    edgeDescriptor e2=make_successful(e1.m_target), e3;
    int solution=3;
    if (std::get<0>(GetParam())){
        e3=make_successful(e2.m_target);
    }
    else{
        e3=make_v1_crashed(e2.m_target);
        solution=1;
    }
    vertex_set_direction(e1.m_target, direction);
    vertex_set_direction(e2.m_target, direction);
    vertex_set_direction(e3.m_target, direction);
    vertex_set_phi(e1.m_target, 0);
    vertex_set_phi(e2.m_target, 0);
    vertex_set_phi(e3.m_target, 0);
    vertex_set_phi(e1.m_source, 0);
    EXPECT_EQ(task_vertices(e3.m_target).size(), solution);
}    

INSTANTIATE_TEST_CASE_P(Directions, ConfiguratorTaskVerticesTest, testing::Combine(testing::Bool(), testing::Values(LEFT, RIGHT, DEFAULT)));


/**
 * @brief Typical 2-level expansion (TURN-DEFAULT)
 * 
 */
TEST_P(ConfiguratorEvaluationQueueManagerTest, addToEvaluationQueueDepth2){
    vertexDescriptor v0=make_successful(MOVING_VERTEX, std::get<0>(GetParam())).m_target, v1;    
    eqm.addToEvaluationQueue(evaluationQ, v0, transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), 1);
    EXPECT_EQ(evaluationQ, std::vector<vertexDescriptor>({v0}));
    int solution=1;
    if (std::get<1>(GetParam())==simResult::crashed){
        v1 =make_v1_crashed(v0).m_target;
        solution++;
    }
    else{
        v1=make_successful(v0).m_target;
    }
    eqm.addToEvaluationQueue(evaluationQ, v1, transitionSystem, v);
    EXPECT_EQ(evaluationQ.size(), solution);
}

/**
 * @brief 3-level expansion (TURN-TURN-DEFAULT)
 * 
 */
TEST_P(ConfiguratorEvaluationQueueManagerTest, addToEvaluationQueueDepth3){
    vertexDescriptor v0=make_successful(MOVING_VERTEX, std::get<0>(GetParam())).m_target, v1, v2;
    eqm.addToEvaluationQueue(evaluationQ, v0, transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), 1);
    v1=make_successful(v0, std::get<0>(GetParam())).m_target;
    eqm.addToEvaluationQueue(evaluationQ, v1, transitionSystem, v);
    int solution=1;
    if (std::get<1>(GetParam())==simResult::crashed){
        v2 =make_v1_crashed(v1).m_target;
        solution++;
    }
    else{
        v2=make_successful(v1).m_target;
    }
    eqm.addToEvaluationQueue(evaluationQ, v2, transitionSystem, v);
    EXPECT_EQ(evaluationQ.size(), solution);
}

//NOTE TO SELF: NEED TO UPDATE LAST ADDED TO EVERY LEVEL!

TEST_P(ConfiguratorEvaluationQueueManagerTest, addToEvaluationQueueNoEdge){
    vertexDescriptor v0=make_successful(MOVING_VERTEX, std::get<0>(GetParam())).m_target, v1;
    eqm.addToEvaluationQueue(evaluationQ, v0, transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), 1);
    int solution=2;
    if (std::get<1>(GetParam())==simResult::crashed){
        v1 =make_v1_crashed(MOVING_VERTEX).m_target;
    }
    else{
        v1=make_successful(MOVING_VERTEX).m_target;
    }
    eqm.addToEvaluationQueue(evaluationQ, v1, transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), solution);
}

TEST_P(ConfiguratorTakeBool, simulateModule){
    std::vector<vertexDescriptor> evaluationQ;
    AttentiveConfigurator::EvaluationQueueManager eqm;
    vertexDescriptor v=MOVING_VERTEX;
    transitionSystem[MOVING_VERTEX].direction=STOP;
    std::vector<vertexDescriptor>new_vertices;
    for (int i=0; i<6; i++){
        new_vertices.push_back(boost::add_vertex(transitionSystem));
    }
    std::vector<vertexDescriptor>solution={new_vertices[0], new_vertices[2], new_vertices[5]};
    transitionSystem[new_vertices[0]].direction=DEFAULT;
    add_edge_withPoses(MOVING_VERTEX,new_vertices[0]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[0], transitionSystem, MOVING_VERTEX);    
    EXPECT_EQ(evaluationQ.size(), 1);

    transitionSystem[new_vertices[1]].direction=LEFT;
    add_edge_withPoses(MOVING_VERTEX,new_vertices[1]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[1], transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), 2);

    transitionSystem[new_vertices[2]].direction=DEFAULT;
    add_edge_withPoses(new_vertices[1],new_vertices[2]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[2], transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), 2);

    transitionSystem[new_vertices[3]].direction=RIGHT;
    add_edge_withPoses(MOVING_VERTEX,new_vertices[3]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[3], transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), 3);


    transitionSystem[new_vertices[4]].direction=RIGHT;
    add_edge_withPoses(new_vertices[3],new_vertices[4]);
    int solutionSize=3;
    if (GetParam()){//crashed
        transitionSystem[new_vertices[4]].outcome=simResult::crashed;
        solution.erase(solution.end()-1);
        solution.push_back(new_vertices[3]);
        solution.push_back(new_vertices[4]);
        solutionSize+=1;
    }
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[4], transitionSystem, MOVING_VERTEX);
    EXPECT_EQ(evaluationQ.size(), solutionSize);
    if (!GetParam()){
        transitionSystem[new_vertices[5]].direction=DEFAULT;
        add_edge_withPoses(new_vertices[4],new_vertices[5]);
        eqm.addToEvaluationQueue(evaluationQ, new_vertices[5], transitionSystem, MOVING_VERTEX);
    }
    EXPECT_EQ(evaluationQ.size(), solutionSize);
    EXPECT_EQ(evaluationQ, solution);
}

TEST_F(ConfiguratorEvaluationQueueManagerTest, addFromNonZeroVertex){
    dummy_vertex(MOVING_VERTEX);
    make_module(currentVertex);
    currentVertex=n_vertices()-1;
    auto v1=make_successful(currentVertex, LEFT).m_target;
    eqm.addToEvaluationQueue(evaluationQ, v1, transitionSystem, currentVertex);
    EXPECT_EQ(evaluationQ.size(), 0);
}

TEST_P(ConfiguratorTakeBool, simulateModuleFromNonZero){
    std::vector<vertexDescriptor> evaluationQ;
    AttentiveConfigurator::EvaluationQueueManager eqm;
    make_module(MOVING_VERTEX);
    vertexDescriptor v=n_vertices()-1;
    std::vector<vertexDescriptor>new_vertices;
    for (int i=0; i<6; i++){
        new_vertices.push_back(boost::add_vertex(transitionSystem));
    }
    std::vector<vertexDescriptor>solution={new_vertices[0], new_vertices[2], new_vertices[5]};
    transitionSystem[new_vertices[0]].direction=DEFAULT;
    add_edge_withPoses(v,new_vertices[0]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[0], transitionSystem, v);    
    EXPECT_EQ(evaluationQ.size(), 1);

    transitionSystem[new_vertices[1]].direction=LEFT;
    add_edge_withPoses(v,new_vertices[1]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[1], transitionSystem, v);
    EXPECT_EQ(evaluationQ.size(), 1);

    transitionSystem[new_vertices[2]].direction=DEFAULT;
    add_edge_withPoses(new_vertices[1],new_vertices[2]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[2], transitionSystem, v);
    EXPECT_EQ(evaluationQ.size(), 2);

    transitionSystem[new_vertices[3]].direction=RIGHT;
    add_edge_withPoses(MOVING_VERTEX,new_vertices[3]);
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[3], transitionSystem, v);
    EXPECT_EQ(evaluationQ.size(), 2);


    transitionSystem[new_vertices[4]].direction=RIGHT;
    add_edge_withPoses(new_vertices[3],new_vertices[4]);
    int solutionSize=3;
    if (GetParam()){//crashed
        transitionSystem[new_vertices[4]].outcome=simResult::crashed;
        solution.erase(solution.end()-1);
        solutionSize-=1;
    }
    eqm.addToEvaluationQueue(evaluationQ, new_vertices[4], transitionSystem, v);
    EXPECT_EQ(evaluationQ.size(), 2);
    if (!GetParam()){
        transitionSystem[new_vertices[5]].direction=DEFAULT;
        add_edge_withPoses(new_vertices[4],new_vertices[5]);
        eqm.addToEvaluationQueue(evaluationQ, new_vertices[5], transitionSystem, v);
    }
    EXPECT_EQ(evaluationQ.size(), solutionSize);
    EXPECT_EQ(evaluationQ, solution);
}

INSTANTIATE_TEST_CASE_P(DirectionsAndOutcomes, 
                        ConfiguratorEvaluationQueueManagerTest, 
                        testing::Combine(
                            testing::Values(LEFT, RIGHT), 
                            testing::Values(simResult::successful, simResult::crashed))
                        );

class MovingVertexTest: public ConfiguratorTest{

    void SetUp() override{
        init(generateGoalTask());
        auto e=make_successful(MOVING_VERTEX,LEFT);
        vertex_set_Di(e.m_target, Disturbance(AVOID, b2Vec2(.5,0), 0));
        m_plan.push_back(e.m_target);
        register_tracker(new ClosedLoop_Tracker());
        register_controller(new Wise_Controller());
        change_task();
        currentVertex=e.m_target;
    }

    void TearDown() override{
        transitionSystem=TransitionSystem(1);
        delete task_controller;
        delete tracker;
    }
};

TEST_F(MovingVertexTest, DirectionIsDefault){
    EXPECT_EQ(vertex_get_direction(MOVING_VERTEX), DEFAULT);
    EXPECT_TRUE(vertex_get_Di(MOVING_VERTEX)== controlGoal.get_disturbance());
}

TEST_F(MovingVertexTest, DiIsCurrentDi){
    pre_explore();    
    EXPECT_EQ(vertex_get_direction(MOVING_VERTEX), DEFAULT);
    EXPECT_TRUE(vertex_get_Di(MOVING_VERTEX)== currentTask.get_disturbance());
}

/**
 * @brief Testing calculation of reamining simulation time in reactive mode
 */
TEST_P(ReactiveConfTest, RemainingTime){
    Task t(Disturbance(), GetParam(), b2Transform_zero, true);
    solution/=controlGoal.getAction().getLinearSpeed();
    EXPECT_EQ(remainingSimulationTime(&t), solution);
}

TEST_P(ReactiveConfTest, Simulate){
    Task t(Disturbance(), GetParam(), b2Transform_zero, true);
    b2World world(GRAVITY);
    worldBuilder->makeBody(world, BodyFeatures(b2Transform(b2Vec2(0.67,0), b2Rot(0))));
    simResult result=simulate(t, world);
    if (GetParam()!=DEFAULT){
        solution=20;
    }
    else{
        solution=27;
    }
    EXPECT_EQ(result.step, solution);
}

TEST_P(LogicalCheckPlanTest, IsPlannedTaskOK){
    make_module(MOVING_VERTEX);
    make_module(3);
    m_plan={2, 3, 6};
    vertexDescriptor src=std::get<0>(GetParam());
    State s;
    s.Dn.set_affordance(std::get<1>(GetParam()));
    s.direction=std::get<2>(GetParam());
    if (std::get<3>(GetParam())){
        EXPECT_TRUE(isPlannedTaskOK(src,s).first);
    }
    else{
        EXPECT_FALSE(isPlannedTaskOK(src,s).first);
    }
}

INSTANTIATE_TEST_CASE_P(PlannedTasks, LogicalCheckPlanTest, ::testing::Values(std::tuple<vertexDescriptor, AffordanceIndex, Direction, bool>(3, 0, DEFAULT, true), 
                                                                            std::tuple<vertexDescriptor, AffordanceIndex, Direction,bool>(6, 0, DEFAULT,true)));

INSTANTIATE_TEST_CASE_P(PlannedTasksNotOk, LogicalCheckPlanTest, ::testing::Values(std::tuple<vertexDescriptor, AffordanceIndex, Direction, bool>(3, 1, DEFAULT, false), 
                                                                            std::tuple<vertexDescriptor, AffordanceIndex, Direction,bool>(6, 1, DEFAULT,false)));


INSTANTIATE_TEST_CASE_P(UnplannedTasks, LogicalCheckPlanTest, ::testing::Values(std::tuple<vertexDescriptor, AffordanceIndex, Direction,bool>(2, 0, LEFT,false), 
                                                                            std::tuple<vertexDescriptor, AffordanceIndex,Direction, bool>(5, 0, DEFAULT,false)));


// TEST_P(ReactiveConfTest, AdjustSimulatedTask){
//     currentTask.set_direction(GetParam());
//     boost::add_edge(MOVING_VERTEX, currentVertex, transitionSystem);
//     Task t(Disturbance(), GetParam(), b2Transform_zero, true);
//     CLTrackerTest _tracker;
//     _tracker.setDeltaTransform(t.getAction().getTransform(0.4));
//     register_tracker(&_tracker);
//     EndCriteria ec=t.getEndCriteria();
//     adjust_simulated_task(currentVertex, t);
//     if (GetParam()!=DEFAULT){
//         b2Transform dt=_tracker.getDeltaTransform();
//         ec.angle.set(ec.angle.get_signed()-dt.q.GetAngle());
//     }
//     EXPECT_EQ(t.getEndCriteria().angle.get(), ec.angle.get());
//     EXPECT_EQ(t.getEndCriteria().distance.get(), ec.distance.get());

// }

INSTANTIATE_TEST_CASE_P(Directions, ReactiveConfTest, ::testing::Values(DEFAULT, RIGHT, LEFT));