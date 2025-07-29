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
    EXPECT_EQ(current_vertices.size(), 0);
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
    b2World world(b2Vec2(0,0));
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

TEST_F(ConfiguratorTest, UpdateGraph){
    Disturbance Di(PURSUE, b2Vec2(0.81, 0.23)), Dn(AVOID, b2Vec2(0.22, 0)), goal(PURSUE, b2Vec2(1.0, 0));
    init(Task(goal, UNDEFINED));
    b2Transform deltaPose(b2Vec2(.5, .27), b2Rot(M_PI_4));
    dummy_vertex(MOVING_VERTEX);
    transitionSystem[1].Di=Di;
    transitionSystem[1].Dn=Dn;
    transitionSystem[1].direction=DEFAULT;
    currentTask=Task(Dn, DEFAULT, b2Transform_zero, true);
    update_graph(transitionSystem, deltaPose);
    EXPECT_FALSE(transitionSystem[1].Di.pose()==Di.pose());
    EXPECT_FALSE(transitionSystem[1].Dn.pose()==Dn.pose());
    EXPECT_FALSE(controlGoal.get_disturbance().pose()==goal.pose());
    EXPECT_TRUE(currentTask.get_disturbance().pose()==Dn.pose());
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
    b2Transform deltaPose=GetParam();
    ConfiguratorTest::Manual_WiseController controller;
    register_controller(&controller);
    controller.set_disturbance(obstacle);
    controller.set_Di_to_goal(goal.get_disturbance());
    vertexDescriptor v1;
    vertex_set_options(0, std::vector<Direction>(DEFAULT));
    add_vertex_now(MOVING_VERTEX, v1, goal.get_disturbance());
    m_plan={v1};
    math::MulT(deltaPose, deltaPose);
    update_graph(transitionSystem, deltaPose);
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
    //b2Transform start=std::get<0>(GetParam());
    vertexDescriptor v1=make_v1_crashed(MOVING_VERTEX, std::get<0>(GetParam()), std::get<1>(GetParam()), std::get<2>(GetParam())).m_target;
    std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, currentVertex);
    b2Vec2 endPosition=std::get<1>(GetParam()).p;
    int expected_splitSize=int(endPosition.Length()/(simulationStep+0.00001))+1;
    EXPECT_EQ(split.size(), expected_splitSize);
    int ct=1;
    for (vertexDescriptor v:split){
        float step_size=(transitionSystem[v].endPose.p-transitionSystem[v].start.p).Length();
        EXPECT_LT(step_size, simulationStep+0.00001);
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
    dummy_vertex(MOVING_VERTEX);
    make_module(currentVertex);
    vertexDescriptor v1=n_vertices()-1;
    vertex_set_outcome(v1, simResult::crashed);
    std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, currentVertex);
    int expected_splitSize=int(vertex_get_endPose(v1).p.Length()/(simulationStep+0.00001))+1;
    EXPECT_EQ(split.size(), expected_splitSize);
    
}

INSTANTIATE_TEST_CASE_P(SplitTask, ConfiguratorTest32DT, ::testing::Values(std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.6, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0, 0), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.26, -0.01), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.265, -0.16), b2Rot(0))),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0, 0.6), b2Rot(M_PI_2)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0.2, 0.27), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.75, 0.27), b2Rot(0)), b2Transform_zero),
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

// TEST_P(ConfiguratorTakeBool, RecyclePlan){
//     std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4};
//     vertexDescriptor task_start=DUMMY;
//     make_ts(avoid, desiredPlan, GetParam());    
//     State s=transitionSystem[2];
//     b2Transform shift=b2Mul(transitionSystem[DUMMY].endPose, transitionSystem[currentVertex].endPose);
//     b2Transform shift_start=b2Transform_zero;
//     math::MulT(shift, transitionSystem);
//     EXPECT_EQ(transitionSystem[currentVertex].endPose, b2Transform_zero);
//     iteration=100;
//     resetPhi();
//     VertexMatch vm(StateMatcher::ABSTRACT, 2);
//     auto edge =boost::add_edge(currentVertex, 2, transitionSystem);
//     bool recycled=recycle_plan(currentVertex, currentVertex, task_start, vm.first, shift_start, s.start, edge, m_plan, s.direction);
//     EXPECT_TRUE(recycled);
//     EXPECT_EQ(m_plan, desiredPlan);
// }

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

INSTANTIATE_TEST_CASE_P(Bool, ConfiguratorTakeBool, testing::Bool());

INSTANTIATE_TEST_CASE_P(Backtrack, ConfiguratorBacktrackTest, ::testing::Values(std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform_zero, b2Transform(b2Vec2(0.6, 0), b2Rot(0)), b2Transform_zero),
                                                                   std::tuple<b2Transform, b2Transform, b2Transform>(b2Transform(b2Vec2(0, 0), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.26, -0.01), b2Rot(-M_PI_2)), b2Transform(b2Vec2(0.265, -0.16), b2Rot(0)))));