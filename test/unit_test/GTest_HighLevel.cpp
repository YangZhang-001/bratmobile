#include "test_classes.h"
#include <gtest/gtest.h>

TEST_F(HighLevelTest, Init){
    EXPECT_TRUE(configurator->get_motor_interface()!=(NULL));
    EXPECT_TRUE(configurator->get_lidar_interface()!= NULL);
    EXPECT_TRUE(configurator->get_tracker()!=NULL);
    EXPECT_TRUE(configurator->get_controller()!=NULL);
}

TEST_F(HighLevelTest, AcquireData){
    di.set_folder("../cul_de_sac/");
    di.newScanAvail();
    EXPECT_TRUE(di.has_interface());
    EXPECT_GT(ci.data2fp.size(),0);
    configurator->set_data2fp(ci.data2fp);
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

TEST_F(ConfiguratorTest, TSCleanup){
    transitionSystem=TransitionSystem(5);
    for (int i=1; i<4;i++){
        auto e=boost::add_edge(MOVING_VERTEX, i, transitionSystem);
        transitionSystem[e.first].step=1;
    }
    boost::add_edge(1,1, transitionSystem); //trivial self-edge
    auto e2= boost::add_edge(2,2, transitionSystem); //nontrivial self-edge
    transitionSystem[e2.first].step=1;
    ts_cleanup(transitionSystem, m_plan);
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
    Logger logger=makeLogger();
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
    Logger logger=makeLogger();
    std::cout<<logger.get_fileName()<<std::endl;
}


TEST_P(HighLevelTest, FirstPlan){
    Task goal;
    bool hasGoal=std::get<0>(GetParam()), success=false;
    if (hasGoal){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);

    }
    configurator->init(goal);
    std::string folder=std::get<1>(GetParam());
    get_plan(folder);
    EXPECT_GT(ci.data2fp.size(),0);
    EXPECT_GT(configurator->data_size(),0);
    if (!hasGoal){
        success=configurator->plan_reaches_horizon();
    }
    else{
        success=configurator->plan_reaches_goal();
    }
    EXPECT_GT(configurator->get_plan().size(),1);
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
    int iteration=std::get<2>(GetParam());
    trackFor(iteration);
    std::vector<vertexDescriptor> updated_plan=get_InterruptedPlan(folder,iteration-1, std::get<3>(GetParam())); //map 2
    int vertices_now=configurator->n_vertices();
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    bool success=planned_to_goal || configurator->getGoal().checkEnded(configurator->vertex_get_endPose(configurator->get_current_vertex())).ended;
    EXPECT_TRUE(success);
}

INSTANTIATE_TEST_CASE_P(CulDeSacTurn, HighLevelInterruptTest, testing::Combine(::testing::Values(false), 
                                                                           ::testing::Values(std::string("../cul_de_sac/")),
                                                                           ::testing::Values(1, 10, 12),
                                                                           ::testing::Values(0, 1) ));

INSTANTIATE_TEST_CASE_P(CulDeSacAvoided, HighLevelInterruptTest, testing::Combine(::testing::Values(false), 
                                                                           ::testing::Values(std::string("../cul_de_sac/")),
                                                                           ::testing::Values(30),
                                                                           ::testing::Values(0) ));

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
    EXPECT_GT(configurator->get_plan().size(), 1);
    vertexDescriptor second_last_v=configurator->get_plan()[configurator->get_plan().size()-2];
    vertexDescriptor last_v=configurator->get_plan()[configurator->get_plan().size()-1];
   // if (!std::get<0>(GetParam())){
        shift=configurator->vertex_get_endPose(last_v);
    //}
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

INSTANTIATE_TEST_CASE_P(CulDeSac, HighLevelTest, ::testing::Combine( ::testing::Values(false), ::testing::Values(std::string("../cul_de_sac/")), ::testing::Values(2, 3, 4, 17, 36)));
                                                                  

INSTANTIATE_TEST_CASE_P(Target40, HighLevelTest, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_40cm/")), ::testing::Values(2, 3, 4, 6,17, 36, 89)));

INSTANTIATE_TEST_CASE_P(Target68, HighLevelTest, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_68cm/")), ::testing::Values(2, 3, 4, 6,17, 36, 89)));



// TEST_P(ReactToNoiseTest, NoisyPlan){
//     const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
//     Logger logger=makeLogger(info);
//     configurator->register_logger(&logger);
//     Task goal;
//     if (std::get<0>(GetParam())){
//         goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
//     }
//     configurator->init(goal);
//     std::string folder=std::get<1>(GetParam()), folder2=std::get<2>(GetParam());
//     std::vector<vertexDescriptor> plan= get_plan(folder);
//     int vertices_og=configurator->n_vertices();
//     trackFor(4);
//     std::vector<vertexDescriptor> updated_plan=get_plan(folder2, std::get<3>(GetParam())); //map 2
//     int vertices_now=configurator->n_vertices();
//     EXPECT_GE(vertices_now, vertices_og);

//     bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
//     bool success=planned_to_goal || configurator->getGoal().checkEnded(configurator->vertex_get_endPose(configurator->get_current_vertex())).ended;
//     EXPECT_TRUE(success);
// }

// INSTANTIATE_TEST_CASE_P(NoisyCombosAvoidance, ReactToNoiseTest, testing::Combine(
//                                                         testing::Values(false),
//                                                         testing::Values("../cul_de_sac/"),
//                                                         testing::Values("../target_40cm/", "../target_68cm/"),
//                                                         testing::Values(2, 3, 6, 11, 17, 39, 89, 97)));



// INSTANTIATE_TEST_CASE_P(NoisyCombosTarget40, ReactToNoiseTest, testing::Combine(
//                                                         testing::Values(true),
//                                                         testing::Values("../target_40cm/"),
//                                                         testing::Values("../cul_de_sac/", "../target_68cm/"),
//                                                         testing::Values(2, 3, 6, 11, 17, 39, 89, 97)));


// INSTANTIATE_TEST_CASE_P(NoisyCombosTarget68, ReactToNoiseTest, testing::Combine(
//                                                         testing::Values(true),
//                                                         testing::Values("../target_68cm/"),
//                                                         testing::Values("../cul_de_sac/", "../target_40cm/"),
//                                                         testing::Values(2, 3, 6, 11, 17, 39, 89, 97)));




int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}

