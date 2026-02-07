#include "test_classes.h"
#include "b2bconfigurator.h"

const bool DEBUG=false;

TEST_F(DebugB2BTest, PreExplore){
    init();
    iteration++;
    pre_explore();
}

TEST_F(DebugB2BTest, Explorer){
    //GTEST_SKIP();
    register_tracker(new ClosedLoop_Tracker);
    iteration++;
    b2World world(GRAVITY);
    dummy_vertex(MOVING_VERTEX);
    explorer(MOVING_VERTEX, transitionSystem, world);
    delete tracker;
}

// TEST_F(DebugB2BTest, ExplorePlan){
//     register_tracker(new ClosedLoop_Tracker);
//     init();
//     EXPECT_FALSE(tracker==NULL);
//     b2World world(GRAVITY);
//     iteration++;
//     explore_plan(world);
//     delete tracker;
// }

// TEST_F(DebugB2BTest, TSCleanup){
//     B2BConfigurator::init();
//     transitionSystem=TransitionSystem(5);
//     for (int i=1; i<4;i++){
//         auto e=boost::add_edge(MOVING_VERTEX, i, transitionSystem);
//         transitionSystem[e.first].step=1;
//     }
//     boost::add_edge(1,1, transitionSystem); //trivial self-edge
//     auto e2= boost::add_edge(2,2, transitionSystem); //nontrivial self-edge
//     transitionSystem[e2.first].step=1;
//     B2BConfigurator::ts_cleanup();
//     EXPECT_EQ(transitionSystem.m_vertices.size(), 4);
//     EXPECT_EQ(boost::out_degree(1, transitionSystem), 0); //out edge deleted
//     EXPECT_EQ(boost::in_degree(1, transitionSystem), 1);
//     EXPECT_EQ(boost::out_degree(2, transitionSystem), 1); //edge is preserved
//     EXPECT_EQ(boost::out_degree(0, transitionSystem), 3);
// }

TEST(FrontierCrashed, predicate){
    TransitionSystem ts(2);
    ts[1].direction=DEFAULT;
    ts[1].outcome=simResult::crashed;
    ts[0].direction=LEFT;
    Frontier f(1, std::vector<vertexDescriptor>{0});
    FrontierCrashed fc(ts, LEFT);
    EXPECT_TRUE(fc(f));
}

TEST(FrontierCrashed, predicate180Turn){
    TransitionSystem ts(3);
    ts[2].direction=DEFAULT;
    ts[2].outcome=simResult::crashed;
    ts[1].direction=LEFT;
    ts[0].direction=LEFT;
    Frontier f(2, std::vector<vertexDescriptor>{0,1});
    FrontierCrashed fc(ts, LEFT);
    EXPECT_TRUE(fc(f));
}



// TEST_F(DebugB2BTest, PartiallyExplore0) {
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionMatrix(MOVING_VERTEX, DEFAULT, MOVING_VERTEX);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 0);
// }

// TEST_F(DebugB2BTest, PartiallyExplore1) {
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionSystem[3].outcome=simResult::crashed;
//     transitionMatrix(MOVING_VERTEX, DEFAULT, MOVING_VERTEX);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 1);
// }

// TEST_F(DebugB2BTest, PartiallyExplore2) {
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionSystem[3].outcome=simResult::crashed;
//     transitionSystem[5].outcome=simResult::crashed;
//     transitionMatrix(MOVING_VERTEX, DEFAULT, MOVING_VERTEX);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 2);
// }

// TEST_F(DebugB2BTest, ApplyTransitionInHindsight){
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionSystem[3].outcome=simResult::crashed;
//     transitionSystem[5].outcome=simResult::crashed;
//     applyTransitionMatrix(MOVING_VERTEX, DEFAULT, false, MOVING_VERTEX, m_plan);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 2);
// }



TEST(ClearVoyance, Add){
    DebugB2BTest::ClearVoyanceTest cv;
    Disturbance d(AVOID), d2(AVOID);
    cv.add(0, d);
    EXPECT_EQ(cv.size(), 1);
    cv.add(0, d2);
    EXPECT_EQ(cv.size(), 1);
    cv.add(1, d);
    EXPECT_EQ(cv.size(), 2);
}

TEST(ClearVoyance, Query){
    DebugB2BTest::ClearVoyanceTest cv;
    Disturbance d(AVOID);
    b2Transform t=b2Transform(b2Vec2(1.0, 0), b2Rot(0));
    d.setPose(t);
    cv.add(0, d);
    EXPECT_EQ(cv.query(0).pose(), t);     
    EXPECT_EQ(cv.query(1).getAffIndex(), NONE); //not found
}

TEST(ClearVoyance, Pop){
    //GTEST_SKIP();
    DebugB2BTest::ClearVoyanceTest cv;
    Disturbance d(AVOID), d2(AVOID);
    b2Transform t=b2Transform(b2Vec2(1.0, 0), b2Rot(0));
    d2.setPose(t);
    cv.add(0, d);
    cv.add(0, d2);
    cv.add(1, d);
    cv.pop(0);
    EXPECT_EQ(cv.query(0).pose(), t);   
    EXPECT_EQ(cv.size(), 2);  
}

TEST_F(DebugB2BTest, AddOptionsHindSight){
    iteration++;
    make_module(MOVING_VERTEX);
    setAllVisited();
    transitionSystem[3].outcome=simResult::crashed;
    std::vector <Direction> options={DEFAULT, LEFT, RIGHT};
    addOptionsInHindsight(MOVING_VERTEX, 2, 3);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 1);
}

TEST_F(DebugB2BTest, Add180TurnToClearvoyance){
    iteration++;
    auto v0=make_successful(MOVING_VERTEX, LEFT).m_target;
    auto v1=make_v1_crashed(v0, b2Transform_zero, b2Transform_zero, generateGoal().pose()).m_target;
    setAllVisited();
    addOptionsInHindsight(MOVING_VERTEX, v0,v1);
    EXPECT_EQ(transitionSystem[v0].options.size(), 0);
    EXPECT_TRUE(clearvoyance.query(v0).isValid());
}

class DebugB2BTestSplit : public DebugB2BTest, public ::testing::WithParamInterface<std::tuple<bool, Direction>> {
};

TEST_P(DebugB2BTestSplit, splitTask){
    b2Transform t=b2Transform(b2Vec2(0.9, 0), b2Rot(0));
    vertexDescriptor v1=TransitionSystem::null_vertex();
    int solution=2;
    dummy_vertex(MOVING_VERTEX);
    if (std::get<0>(GetParam())){
        v1=make_successful(currentVertex, std::get<1>(GetParam())).m_target;   
            solution=1; //default direction is not split
    }
    else{
        v1=make_v1_crashed(currentVertex, b2Transform_zero, t, t).m_target;
    }
    vertex_set_direction(v1, std::get<1>(GetParam()));
    std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, currentVertex);
    EXPECT_EQ(split.size(), solution);
}

// TEST_F(DebugB2BTest, splitTaskSuccess){
//     b2Transform t=b2Transform(b2Vec2(0.6, 0), b2Rot(0));
//     vertexDescriptor v1=make_successful(MOVING_VERTEX).m_target;
//     std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, currentVertex);
//     EXPECT_EQ(split.size(), 1);
// }

// TEST_F(DebugB2BTest, splitTaskTurn){
//     b2Transform t=b2Transform(b2Vec2(0.6, 0), b2Rot(0));
//     vertexDescriptor v1=make_v1_crashed(MOVING_VERTEX, b2Transform_zero, t, t).m_target;
//     std::vector <vertexDescriptor> split =splitTask(v1, transitionSystem[v1].direction, currentVertex);
//     EXPECT_EQ(split.size(), 2);
// }

INSTANTIATE_TEST_CASE_P(SplitTask, DebugB2BTestSplit, ::testing::Combine(
    ::testing::Bool(), 
    ::testing::Values(DEFAULT, LEFT, RIGHT)));

TEST_P(B2BTestGetGoal, GetDisturbanceGoal){
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

INSTANTIATE_TEST_CASE_P(DisturbanceIsGoal, B2BTestGetGoal, ::testing::Values(
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), DEFAULT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), DEFAULT, RIGHT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), LEFT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.82, 0.31), b2Rot(0)), RIGHT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, RIGHT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.80, 0.0), b2Rot(-M_PI_2)), DEFAULT, LEFT)));


TEST_P(B2BTestGetObstacle, GetDisturbanceObstacle){
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

INSTANTIATE_TEST_CASE_P(DisturbanceIsObstacle, B2BTestGetObstacle, ::testing::Values(
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.4, 0.0), b2Rot(M_PI_2)), LEFT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.4, 0.0), b2Rot(M_PI_2)), RIGHT, DEFAULT),
                                                                   std::tuple<b2Transform,Direction, Direction>(b2Transform(b2Vec2(0.40, 0.31), b2Rot(-M_PI_2)), RIGHT, DEFAULT)));

/**
 * @brief Crash on the way to goal
 * 
 */
TEST_F(B2BTestGetObstacle, CrashToGoal){
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

TEST_F(B2BTestGetObstacle, AvoidNoGoal){
    init(Task());
    EXPECT_FALSE(controlGoal.get_disturbance().isValid()); //test case health check
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

TEST_F(DebugB2BTest, ClearVoyance){
    //GTEST_SKIP();
    init(Task());
    EXPECT_FALSE(controlGoal.get_disturbance().isValid()); //test case health check
    EXPECT_EQ(controlGoal.get_disturbance().getAffIndex(), NONE);
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05), bf2=bodyFeatures(0, .55, 0, 0.02, 0.05), bf3=bodyFeatures(0, -0.55, 0, 0.02, 0.05);
    //bf.attention=1;
    make_module(MOVING_VERTEX);
    transitionSystem[MOVING_VERTEX].Di=Disturbance(bf); //current task was avoiding
    transitionSystem[MOVING_VERTEX].Di.validate();
    transitionSystem[3].Dn=Disturbance(bf2); //obstacle on the left
    transitionSystem[3].outcome=simResult::crashed; //obstacle on the left
    transitionSystem[5].outcome=simResult::crashed; //obstacle on the left
    transitionSystem[5].Dn=Disturbance(bf3); //obstacle on the right
    transitionSystem[3].Dn.validate();
    transitionSystem[5].Dn.validate();
    setAllVisited();
    clearvoyance.add(MOVING_VERTEX, transitionSystem[3].Dn);
    clearvoyance.add(MOVING_VERTEX, transitionSystem[5].Dn);
   // Disturbance solution=transitionSystem[MOVING_VERTEX].Di;
    transitionSystem[MOVING_VERTEX].direction=STOP;
    vertex_options_push_back(MOVING_VERTEX, DEFAULT);
    vertex_options_push_back(MOVING_VERTEX, DEFAULT);
    Disturbance Di= getDisturbance(transitionSystem, MOVING_VERTEX, world, DEFAULT, transitionSystem[MOVING_VERTEX].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, transitionSystem[3].Dn.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, transitionSystem[3].Dn.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), transitionSystem[3].Dn.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, transitionSystem[3].Dn.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, transitionSystem[3].Dn.bf.halfWidth);
    //NEED TO MAKE METHOD TO CHECK THAT DISTUBRANCE IS NOT BEING POPPED OFF (MOCK?)
    clearvoyance.pop(MOVING_VERTEX);
    Di= getDisturbance(transitionSystem, MOVING_VERTEX, world, DEFAULT, transitionSystem[MOVING_VERTEX].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, transitionSystem[5].Dn.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, transitionSystem[5].Dn.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), transitionSystem[5].Dn.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, transitionSystem[5].Dn.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, transitionSystem[5].Dn.bf.halfWidth);
}

TEST_F(DebugB2BTest, BacktrackCollision){
    init(Task());
    dummy_vertex(MOVING_VERTEX);
    vertexDescriptor v1= make_v1_crashed(currentVertex).m_target;
    std::vector <vertexDescriptor> evaluation_q={v1}, priority_q, plan_prov;
    std::set <vertexDescriptor> closed;
    setAllVisited();
    EXPECT_TRUE(transitionSystem[currentVertex].visited());
    backtrack(evaluation_q, priority_q, closed, plan_prov, currentVertex, currentVertex);
    EXPECT_FALSE(std::find_if(priority_q.begin(), priority_q.end(), 
        [this](vertexDescriptor v){return v==1;})==priority_q.end());
}

// TEST_F(ConfiguratorTest, BacktrackCollision){
//     init(Task());
//     dummy_vertex(MOVING_VERTEX);
//     make_v1_crashed(currentVertex);
//     transitionSystem[currentVertex].endPose.p.x=0.26;
//     std::vector <vertexDescriptor> evaluation_q, priority_q, plan_prov;
//     std::set <vertexDescriptor> closed;
//     setPhi(transitionSystem[currentVertex]);
//     EXPECT_TRUE(transitionSystem[currentVertex].visited());
//     backtrack(evaluation_q, priority_q, closed, plan_prov, currentVertex, currentVertex);
//     EXPECT_FALSE(std::find_if(priority_q.begin(), priority_q.end(), 
//         [this](vertexDescriptor v){return v==1;})==priority_q.end());
// }

TEST_F(HighLevelTestB2B, Init){
    EXPECT_TRUE(configurator->get_motor_interface()!=(NULL));
    EXPECT_TRUE(configurator->get_tracker()!=NULL);
    EXPECT_TRUE(configurator->get_controller()!=NULL);
}

TEST_F(HighLevelTestB2B, AcquireData){
    di.set_folder("../cul_de_sac/");
    di.newScanAvail();
    EXPECT_TRUE(di.has_interface());
    EXPECT_GT(configurator->data_size(),0);
}

TEST_P(HighLevelTestB2B, FirstPlanB2B){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=HighLevelTest::makeLogger(info);
    configurator->register_logger(&logger);
    Task goal;
    bool hasGoal=std::get<0>(GetParam()), success=false;
    if (hasGoal){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    }
    configurator->init(goal);
    std::string folder=std::get<1>(GetParam());
    get_plan(folder);
    EXPECT_GT(configurator->data_size(),0);
    if (!hasGoal){
        success=configurator->plan_reaches_horizon();
    }
    else{
        success=configurator->plan_reaches_goal();
    }
    EXPECT_GE(configurator->get_plan().size(),1);
    EXPECT_TRUE(success);
}

TEST_P(HighLevelTestB2B, CheckPlanB2B){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=HighLevelTest::makeLogger(info);
    configurator->register_logger(&logger);
    Task goal;
    if (std::get<0>(GetParam())){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    }
    configurator->init(goal);
    std::string folder=std::get<1>(GetParam());
    std::vector<vertexDescriptor> plan= get_plan(folder);
    int vertices_og=configurator->n_vertices();
    int iteration=std::get<2>(GetParam());
    trackFor(iteration);
    std::vector<vertexDescriptor> updated_plan=get_plan(folder, iteration-1); //map 2
    EXPECT_EQ(di.get_iteration(), iteration);
    int vertices_now=configurator->n_vertices();
    EXPECT_LE(vertices_now, vertices_og);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    bool success=planned_to_goal || configurator->getGoal().checkEnded(configurator->vertex_get_endPose(configurator->get_current_vertex())).ended;
    EXPECT_TRUE(success);
}

TEST_P(HighLevelTestB2B, RecycleB2B){
    GTEST_SKIP();
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=HighLevelTest::makeLogger(info);
    configurator->register_logger(&logger);
    Task goal;
    b2Transform shift=b2Transform_zero;
    if (std::get<0>(GetParam())){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0, 0), 0),DEFAULT);
    }
    configurator->init(goal);
    std::string folder=std::get<1>(GetParam());
    std::vector<vertexDescriptor> plan= get_plan(folder), finished_plan;
    EXPECT_GE(configurator->get_plan().size(), 1);
    vertexDescriptor second_last_v=configurator->get_plan()[configurator->get_plan().size()-2];
    vertexDescriptor last_v=configurator->get_plan()[configurator->get_plan().size()-1];
    shift=configurator->vertex_get_endPose(last_v);
    int vertices_og=configurator->n_vertices();
    configurator->addIteration(100);
    configurator->set_current_v(last_v); //simulate plan finished
    configurator->getTask().set_change(true);
    configurator->set_plan({});
    wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(), finished_plan);
    configurator->getTask().set_change(true);
    EXPECT_EQ(configurator->getTask().get_direction(), configurator->vertex_get_direction(last_v));
    EXPECT_TRUE(configurator->getTask().get_disturbance()==configurator->vertex_get_Di(last_v));
    EXPECT_EQ(configurator->get_current_vertex(), last_v);
    math::MulT(shift, configurator->get_ts());
    b2Transform newStart=configurator->vertex_get_endPose(last_v);
    EXPECT_LT(newStart.p.Length(),0.0001);
    EXPECT_LT(newStart.q.GetAngle(),0.0001);
    std::vector<vertexDescriptor> updated_plan=get_plan(folder); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_LE(vertices_now, vertices_og);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);
}


INSTANTIATE_TEST_CASE_P(CulDeSac, HighLevelTestB2B, ::testing::Combine( ::testing::Values(false), ::testing::Values(std::string("../cul_de_sac/")), ::testing::Values(2, 3, 4, 17, 36)));
                                                                  

INSTANTIATE_TEST_CASE_P(Target40, HighLevelTestB2B, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_40cm/")), ::testing::Values(2, 3, 4, 6,17, 89)));

INSTANTIATE_TEST_CASE_P(Target68, HighLevelTestB2B, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_68cm/")), ::testing::Values(2, 3, 4, 6, 17, 36)));


TEST_P(DebugB2BTestVertex, ClearVoyanceTurn){ //test clearvoyance when turning on the spot
    //GTEST_SKIP();
    init(Task());
    EXPECT_FALSE(controlGoal.get_disturbance().isValid()); //test case health check
    EXPECT_EQ(controlGoal.get_disturbance().getAffIndex(), NONE);
    b2World world(b2Vec2(0,0));
    BodyFeatures bf=bodyFeatures(.55, 0, 0, 0.02, 0.05), bf2=bodyFeatures(0, .55, 0, 0.02, 0.05), bf3=bodyFeatures(0, -0.55, 0, 0.02, 0.05);
    //bf.attention=1;
    make_module(MOVING_VERTEX); //no dummy
    vertexDescriptor v0=GetParam(), v1=v0+1;;
    transitionSystem[MOVING_VERTEX].Di=Disturbance(bf); //current task was avoiding
    transitionSystem[MOVING_VERTEX].Di.validate();
    transitionSystem[v1].Dn=Disturbance(bf2); //obstacle on the left
    transitionSystem[v1].Dn.validate();
    transitionSystem[v1].outcome=simResult::crashed;
    setAllVisited();
    clearvoyance.add(v0, transitionSystem[v1].Dn);
   // Disturbance solution=transitionSystem[MOVING_VERTEX].Di;
    transitionSystem[MOVING_VERTEX].direction=STOP;
    vertex_options_push_back(v0, vertex_get_direction(GetParam())); //add option same turn
    Disturbance Di= getDisturbance(transitionSystem, v0, world, vertex_get_direction(GetParam()), transitionSystem[v0].endPose);
    EXPECT_EQ(Di.bf.pose.p.x, transitionSystem[v1].Dn.bf.pose.p.x);
    EXPECT_EQ(Di.bf.pose.p.y, transitionSystem[v1].Dn.bf.pose.p.y);
    EXPECT_EQ(Di.bf.pose.q.GetAngle(), transitionSystem[v1].Dn.bf.pose.q.GetAngle());
    EXPECT_EQ(Di.bf.halfLength, transitionSystem[v1].Dn.bf.halfLength);
    EXPECT_EQ(Di.bf.halfWidth, transitionSystem[v1].Dn.bf.halfWidth);
    //NEED TO MAKE METHOD TO CHECK THAT DISTUBRANCE IS NOT BEING POPPED OFF (MOCK?)
    clearvoyance.pop(MOVING_VERTEX);
}

INSTANTIATE_TEST_CASE_P(TurningVertices, DebugB2BTestVertex, ::testing::Values(2, 4));

TEST_F(HighLevelTestB2B, TrickyScenarioB2B){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    configurator->init(DebugConfigurator::generateGoalTask());
    configurator->addIteration();
    configurator->get_worldbuilder()->add_iteration();
    configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTricky());
    b2World world(GRAVITY);
    configurator->explorePlan(world);
    EXPECT_GT(configurator->get_plan().size(), 0);
    EXPECT_TRUE(has180Turn(configurator->get_plan()));
    EXPECT_FALSE(configurator->get_plan().empty());
    if (!configurator->get_plan().empty()){
        bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
        EXPECT_TRUE(planned_to_goal);        
    }
}

TEST_F(HighLevelTestB2B, TrappedB2B){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    configurator->init(DebugConfigurator::generateGoalTask());
    configurator->addIteration();
    configurator->get_worldbuilder()->add_iteration();
    configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTrickyTrap(.35));
    b2World world(GRAVITY);
    configurator->explorePlan(world);
    EXPECT_GT(configurator->get_plan().size(), 0);
    EXPECT_TRUE(has180Turn(configurator->get_plan()));
    EXPECT_FALSE(configurator->get_plan().empty());
    if (!configurator->get_plan().empty()){
        bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
        EXPECT_TRUE(planned_to_goal);        
    }
}