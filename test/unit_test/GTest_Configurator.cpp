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

/**
 * @brief Parameters: robot position, previous task direction, current task direction
 * 
 */
class ConfiguratorTestGetGoal:public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, Direction,Direction>>{
    protected:
    /**
     * @brief Sets up vertex for testing using the parameters
     * 
     * @param v 
     */
    void vertex_setup(vertexDescriptor v, const Disturbance & Di, const Disturbance &Dn=Disturbance()){
        transitionSystem[v].direction=std::get<1>(GetParam());
        transitionSystem[v].endPose=std::get<0>(GetParam());
        vertex_options_push_back(v, std::get<2>(GetParam()));
        transitionSystem[v].Di=Di; 
        transitionSystem[v].Di.validate();
        transitionSystem[v].Dn=Dn; 
        if (Dn.getAffIndex()!=NONE){
            transitionSystem[v].Dn.validate();  
        }
    }

    void SetUp(){
        Disturbance goal(PURSUE, b2Vec2(1.0, 0));
        init(Task(goal, UNDEFINED));
        data2fp.emplace(Pointf(0.55, 0)); //make point corresponding to obstacle
        dummy_vertex(MOVING_VERTEX);
    }

    void TearDown(){
        controlGoal=Task();
        boost::remove_vertex(currentVertex, transitionSystem);
    }
};

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

class ConfiguratorTestGetObstacle: public ConfiguratorTestGetGoal{
    protected:
    void SetUp(){
        Disturbance goal(PURSUE, b2Vec2(1.0, 0));
        init(Task(goal, UNDEFINED));
        data2fp.emplace(Pointf(0.55, 0)); //make point corresponding to obstacle
        dummy_vertex(MOVING_VERTEX);
    }

    void TearDown(){
        controlGoal=Task();
        boost::remove_vertex(currentVertex, transitionSystem);
    }
};

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
    applyTransitionMatrix(e.m_target, std::get<1>(GetParam()), false, currentVertex, plan);
    int expected=expectedOptions(std::get<1>(GetParam()), std::get<2>(GetParam()), target);
    EXPECT_EQ(transitionSystem[e.m_target].options.size(), expected);
}


/**
 * @brief Simulates finding option for task executing
 * 
 */
TEST_P(ConfiguratorTestTransitionMatrix, InPlanNotVisited){
    currentTask.set_direction(std::get<1>(GetParam()));
    vertex_set_direction(currentVertex, std::get<1>(GetParam()));
    if (std::get<1>(GetParam())==UNDEFINED || std::get<1>(GetParam())==STOP){
        return;
    }
    //dummy_vertex(MOVING_VERTEX);
    currentVertex = boost::add_vertex(transitionSystem);
    make_module(currentVertex);
    planIsDirection(std::get<1>(GetParam()));
    currentVertex=MOVING_VERTEX;
    current_vertices={currentVertex};
    currentTask.set_change(false);
    currentTask.setMotorStep(20);
    iteration=2;
    applyTransitionMatrix(MOVING_VERTEX, currentTask.get_direction(), false, MOVING_VERTEX, plan);
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
    transitionSystem[e.m_target].direction=transitionSystem[plan[0]].direction;
    setPhi(transitionSystem[e.m_target]);
    applyTransitionMatrix(MOVING_VERTEX, transitionSystem[e.m_target].direction, false, MOVING_VERTEX, plan);
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
 * GRAPH looking like this
 * 
 *      *             
                     v3(LEFT)---v4(DEFAULT)                      v13(LEFT)*---v14(DEFAULT)*
                    /                                           /
                   v1 --- v2(DEFAULT)       v8(LEFT)*---v9(DEFAULT)* --- v12(DEFAULT)
                    \                       /                   \
                    v5(RIGHT)* --- v6(DEFAULT)* --- v7(DEFAULT)    v15(RIGHT) --- v16(DEFAULT)  
                                            \
                                              v10(RIGHT) --- v11(DEFAULT)        
                                                       
                                                    

        *                  
 * 
 */
TEST_F(ConfiguratorTest, RecycleLongPlan){
    HorizonStarPlanner planner;
    register_planner(&planner);
    b2Transform shift, shift_start=b2Transform_zero;
    shift.p.x=1;
    Disturbance obstacle(AVOID, b2Vec2(.6,0)), goal(PURSUE, shift.p);
    init(Task(goal,DEFAULT));
    dummy_vertex(MOVING_VERTEX);
    vertexDescriptor task_start=currentVertex; //1
    make_module(currentVertex);
    make_module(n_vertices()-1);
    make_module(n_vertices()-1);
    currentVertex=14; //see sketch
    std::vector<vertexDescriptor> all(n_vertices()-1), avoid={3, 4, 5, 6}, safe(n_vertices()-7), desiredPlan={1,5, 6, 8, 9, 13, 14};
    std::iota(all.begin(), all.end(), 1);
    std::iota(safe.begin(), safe.end(), 7);
    vertex_set_Dn(2, obstacle);
    set_Di(avoid, obstacle);
    set_Di(safe, goal);
    vertex_set_Di(2, goal);
    State s=transitionSystem[2];
    currentTask.setMotorStep(0);
    currentTask.set_change(1);
    math::applyAffineTrans(shift, transitionSystem);
    iteration=100;
    EXPECT_EQ(vertex_get_endPose(currentVertex), b2Transform_zero);
    VertexMatch vm(StateMatcher::ABSTRACT, 2);
    auto edge =boost::add_edge(currentVertex, 2, transitionSystem);
    bool recycled=recycle_plan(currentVertex, currentVertex, task_start, vm.first, shift_start, s.start, edge, plan, s.direction);
    EXPECT_TRUE(recycled);
    EXPECT_EQ(plan, desiredPlan);
}

TEST_F(ConfiguratorTest, RecycleShortPlan){
    HorizonStarPlanner planner;
    register_planner(&planner);
    b2Transform shift, shift_start=b2Transform_zero;
    shift.p.x=1;
    Disturbance obstacle(AVOID, b2Vec2(.6,0));
    dummy_vertex(MOVING_VERTEX);
    vertexDescriptor task_start=currentVertex; //1
    make_module(currentVertex);
    currentVertex=n_vertices()-1; //see sketch
    std::vector<vertexDescriptor>  avoid={3, 5};
    vertex_set_Dn(2, obstacle);
    set_Di(avoid, obstacle);
    State s=transitionSystem[2];
    currentTask.setMotorStep(0);
    currentTask.set_change(1);
    math::applyAffineTrans(shift, transitionSystem);
    iteration=100;
    EXPECT_EQ(vertex_get_endPose(currentVertex), b2Transform_zero);
    VertexMatch vm(StateMatcher::ABSTRACT, 2);
    auto edge =boost::add_edge(currentVertex, 2, transitionSystem);
    bool recycled=recycle_plan(currentVertex, currentVertex, task_start, vm.first, shift_start, s.start, edge, plan, s.direction);
    EXPECT_TRUE(recycled);
    EXPECT_EQ(plan, desiredPlan);
}