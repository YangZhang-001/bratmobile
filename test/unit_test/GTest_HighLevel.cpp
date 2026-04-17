#include "test_classes.h"
#include <gtest/gtest.h>
const bool DEBUG=false;


TEST_F(HighLevelTest, Init){
    EXPECT_TRUE(configurator->get_motor_interface()!=(NULL));
    EXPECT_TRUE(configurator->get_tracker()!=NULL);
    EXPECT_TRUE(configurator->get_controller()!=NULL);
}

TEST_F(HighLevelTest, AcquireData){
    di.set_folder("../cul_de_sac/");
    di.newScanAvail();
    EXPECT_TRUE(di.has_interface());
    EXPECT_GT(configurator->get_data2fp().size(),0);
    EXPECT_GT(configurator->data_size(),0);
}

TEST(Initialisation, InitialMap){
    DebugConfigurator configurator_tmp;
    EXPECT_EQ(configurator_tmp.n_vertices(),1);
    EXPECT_EQ(configurator_tmp.n_edges(), 0);
}

TEST(Boost, RemoveNoEdgesIf){
    TransitionSystem ts(3);
    vertexDescriptor v=2;
    boost::remove_out_edge_if(MOVING_VERTEX, is_not_v(v), ts);
    EXPECT_EQ(boost::out_degree(MOVING_VERTEX, ts), 0);
}

TEST(Boost, RemoveoneEdgeIf){
    TransitionSystem ts(3);
    vertexDescriptor v=2;
    boost::add_edge(MOVING_VERTEX, 2, ts);
    boost::add_edge(MOVING_VERTEX,1, ts);
    boost::remove_out_edge_if(MOVING_VERTEX, is_not_v(v), ts);
    EXPECT_EQ(boost::out_degree(MOVING_VERTEX, ts), 1);
    EXPECT_EQ(boost::in_degree(2, ts), 1);
}

TEST_P(HighLevelInterruptTestTest, GenerateInterruptPoint){
    std::vector <Direction> allDirections={LEFT, RIGHT, DEFAULT};
    std::vector<bool> collided;
    Disturbance disturbance(AVOID, b2Vec2(0.7, 0));
    configurator->dummy_vertex(MOVING_VERTEX);
    configurator->vertex_set_direction(configurator->get_current_vertex(), GetParam());
    if (GetParam()==DEFAULT){
        configurator->vertex_set_endPose(configurator->get_current_vertex(), b2Transform(b2Vec2(0.5, 0), b2Rot(0)));
    }
    else {
        float angle=M_PI_2;
        if (GetParam()==RIGHT){
            angle=-angle;
        }
        configurator->vertex_set_endPose(configurator->get_current_vertex(), b2Transform(b2Vec2(0,0), b2Rot(angle)));
    }
    Pointf obstacle(disturbance.getPosition().x, disturbance.getPosition().y);
    Pointf newObstacle=generateInterruptingPoint(-1);
    configurator->set_data2fp({obstacle, newObstacle});
    BodyFeatures bf1(b2Transform(disturbance.getPosition(), b2Rot(0))), bf2(b2Transform(b2Vec2(newObstacle.x, newObstacle.y),b2Rot(0)));
    b2World world(GRAVITY);
    configurator->get_worldbuilder()->set_world_objects({bf2});
    for (Direction d: allDirections){
        Task task(disturbance, d, configurator->vertex_get_start(configurator->get_current_vertex()),true);
        EXPECT_EQ(task.get_direction(), d);
        configurator->get_worldbuilder()->buildWorld(world, b2Transform_zero, d);        
        simResult sim=configurator->simulate(task, world);
        EXPECT_GT(sim.step, 0);
        if (task.get_direction()==GetParam()){
            EXPECT_EQ(sim.resultCode, simResult::crashed);
            EXPECT_NEAR(sim.collision.getPosition().x, newObstacle.x, 0.01);
            EXPECT_NEAR(sim.collision.getPosition().y, newObstacle.y, 0.01);
        }
        else{
            EXPECT_NE(sim.resultCode, simResult::crashed);
            EXPECT_EQ(sim.collision.getPosition().x, 10000);
            EXPECT_EQ(sim.collision.getPosition().y, 10000);

        }    
        if (::testing::Test::HasFailure()){
            std::cout<<"Failed with direction:"<<d<<std::endl;
        }
    }

    
}

INSTANTIATE_TEST_CASE_P(TaskDirections, HighLevelInterruptTestTest, testing::Values(LEFT, RIGHT, DEFAULT));

TEST_F(ConfiguratorTest, TSCleanup){
    transitionSystem=TransitionSystem(5);
    for (int i=1; i<4;i++){
        auto e=boost::add_edge(MOVING_VERTEX, i, transitionSystem);
        transitionSystem[e.first].step=1;
        transitionSystem[e.first].it_observed=1;
    }
    boost::add_edge(1,1, transitionSystem); //trivial self-edge
    auto e2= boost::add_edge(2,2, transitionSystem); //nontrivial self-edge
    transitionSystem[e2.first].it_observed=1;
    transitionSystem[e2.first].step=1;
    ts_cleanup();
    EXPECT_EQ(transitionSystem.m_vertices.size(), 4);
    EXPECT_EQ(boost::out_degree(1, transitionSystem), 0); //out edge deleted
    EXPECT_EQ(boost::in_degree(1, transitionSystem), 1);
    EXPECT_EQ(boost::out_degree(2, transitionSystem), 1); //edge is preserved
    EXPECT_EQ(boost::out_degree(0, transitionSystem), 3);
}


TEST(Boost, CopyGraph){
    TransitionSystem g1(5), g2;
    for (int i=1; i<4;i++){
        boost::add_edge(0, i, g1);
    }
    boost::copy_graph(g1, g2);
    //just checking if it segfaults
}

TEST(Boost, CopyFTS){
    TransitionSystem g1(5), g2;
    for (int i=1; i<4;i++){
        boost::add_edge(0, i, g1);
    }
    FilteredTS fts(g1, ViableEdge(&g1), Connected(&g1));
    boost::copy_graph(fts, g2);
    EXPECT_EQ(g2.m_vertices.size(), 4);
}

TEST_F(HighLevelTest, ParseFolder){
    std::string str("\"../hello../\""), result;
    try{
        result=parseFolder(str);
    }
    catch (std::exception &e){
        std::cout<<e.what()<<std::endl;
    }
    EXPECT_EQ(result, "/hello..");
}

TEST_F(HighLevelTest, ParseIteration){
    std::string str("(false, \"../cul_de_sac/\", 2)"), result;
    try{
        result=parseIteration(str);
    }
    catch (std::exception &e){
        std::cout<<e.what()<<std::endl;
    }
    EXPECT_EQ(result, "2");
}

TEST_F(HighLevelTest, MakeLogger){
    Logger logger=makeLogger("");
    std::cout<<logger.get_fileName()<<std::endl;
}

TEST_F(HighLevelTest, MakeLoggerParse){
    Logger logger=makeLogger("../test../");
    std::cout<<logger.get_fileName()<<std::endl;
}

TEST_F(HighLevelTest, MakeLoggerString){
    std::string folder("../test../");
    Logger logger=makeLogger(folder.c_str());
    std::cout<<logger.get_fileName()<<std::endl;
}

TEST_P(HighLevelTest, MakeLoggerParseInfo){
    const char* info=testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
}

TEST_F(ReactToNoiseTest, ParseFolder){
    std::string str("../i_am/, /test/");    
    std::string result=parseFolder(str);
    EXPECT_EQ(result, "/i_am/, /test");    
}

TEST_F(ReactToNoiseTest, CarveIteration){
    std::string str("/i_am/, /test");    
    std::pair<std::string, std::string> result=carveScenario(str);
    EXPECT_EQ(result.first, "i_am");    
    EXPECT_EQ(result.second, "test");    
}

TEST_F(ReactToNoiseTest, MakeLogger){
    Logger logger=makeLogger("");
    std::cout<<logger.get_fileName()<<std::endl;
}


TEST_P(HighLevelTest, FirstPlan){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
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


TEST_P(HighLevelTest, CheckPlan){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
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

TEST_P(HighLevelInterruptTest, CheckNoisyPlan){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    Task goal;
    if (std::get<0>(GetParam())){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    }
    configurator->init(goal);
    std::string folder=std::get<1>(GetParam());
    std::vector<vertexDescriptor> plan= get_plan(folder);
    int vertices_og=configurator->n_vertices();
    int iteration=std::get<2>(GetParam()), taskToInterrupt=std::get<3>(GetParam());
    trackFor(iteration);
    Pointf interruptingPoint;
    std::vector<vertexDescriptor> updated_plan=get_InterruptedPlan(folder,iteration-1, taskToInterrupt, &interruptingPoint); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_GT(vertices_now, vertices_og);    
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    bool success=planned_to_goal || configurator->getGoal().checkEnded(configurator->vertex_get_endPose(configurator->get_current_vertex())).ended;
    Disturbance interruptingDisturbance(AVOID, b2Vec2(interruptingPoint.x, interruptingPoint.y), 0);
    b2World world(GRAVITY);
    configurator->get_worldbuilder()->buildWorld(world, b2Transform_zero, DEFAULT);
    Robot robot(&world); 
    if (overlaps(robot.box(), &interruptingDisturbance)){
        EXPECT_TRUE(configurator->get_plan().size()==0);
        EXPECT_FALSE(success);
    }
    else{
    }        
    EXPECT_TRUE(success);
}


// /**
//  * @brief The robot is boxed into a cul de sac
//  * 
//  */
// TEST_F(HighLevelTest, BoxedIn){
//     const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
//     Logger logger=HighLevelTestBase::makeLogger();
//     configurator->register_logger(&logger);
//     configurator->init();
//     configurator->addIteration();
//     configurator->get_worldbuilder()->add_iteration();
//     configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeCulDeSac(0.6, 0.5));
//     b2World world(GRAVITY);
//     configurator->explorePlan(world);
//     EXPECT_GT(configurator->get_plan().size(), 0);
//     bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
//     EXPECT_TRUE(planned_to_goal);
// }

/**
 * @brief The robot cannot pass in a small space between two obstacles and goes around
 * 
 */
TEST_F(HighLevelTest, TrickyScenario){
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

/**
 * @brief The robot cannot pass in a small space between two obstacles and goes around
 * 
 */
TEST_F(HighLevelTest, Trapped){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    configurator->init(DebugConfigurator::generateGoalTask());
    configurator->addIteration();
    configurator->get_worldbuilder()->add_iteration();
    configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTrickyTrap(.35));
    b2World world(GRAVITY);
    configurator->explorePlan(world);
    EXPECT_LE(configurator->get_plan().size(), 0);
    EXPECT_FALSE(has180Turn(configurator->get_plan()));
    EXPECT_TRUE(configurator->get_plan().empty());
    if (!configurator->get_plan().empty()){
        bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
        EXPECT_TRUE(planned_to_goal);        
    }
}

INSTANTIATE_TEST_CASE_P(CulDeSacTurning, HighLevelInterruptTest, testing::Combine(::testing::Values(false), 
                                                                           ::testing::Values(std::string("../cul_de_sac/")),
                                                                           ::testing::Values(2),
                                                                           ::testing::Values(-1, 0) ));
                                                                           //synth fails, debug later!

INSTANTIATE_TEST_CASE_P(CulDeSacAvoided, HighLevelInterruptTest, testing::Combine(::testing::Values(false), 
                                                                           ::testing::Values(std::string("../cul_de_sac/")),
                                                                           ::testing::Values(30),
                                                                           ::testing::Values(-1) ));

TEST_P(HighLevelTest, Recycle){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
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
    configurator->setTask(wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(), finished_plan));
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
    EXPECT_NEAR(vertices_now, vertices_og, 1);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);
}



INSTANTIATE_TEST_CASE_P(CulDeSac, HighLevelTest, ::testing::Combine( ::testing::Values(false), ::testing::Values(std::string("../cul_de_sac/")), ::testing::Values(2, 3, 4, 17, 36)));
                                                                  

INSTANTIATE_TEST_CASE_P(Target40, HighLevelTest, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_40cm/")), ::testing::Values(2, 3, 4, 6,17, 89)));

INSTANTIATE_TEST_CASE_P(Target68, HighLevelTest, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_68cm/")), ::testing::Values(2, 3, 4, 6, 17, 36)));


int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}

