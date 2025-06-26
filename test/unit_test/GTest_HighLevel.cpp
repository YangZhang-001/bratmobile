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
    boost::remove_out_edge_if(movingVertex, is_not_v(v), ts);
    EXPECT_EQ(boost::out_degree(movingVertex, ts), 0);
}

TEST(Boost, RemoveoneEdgeIf){
    TransitionSystem ts(3);
    vertexDescriptor v=2;
    boost::add_edge(movingVertex, 2, ts);
    boost::add_edge(movingVertex,1, ts);
    boost::remove_out_edge_if(movingVertex, is_not_v(v), ts);
    EXPECT_EQ(boost::out_degree(movingVertex, ts), 1);
    EXPECT_EQ(boost::in_degree(2, ts), 1);
}

TEST_F(ConfiguratorTest, TSCleanup){
    transitionSystem=TransitionSystem(5);
    for (int i=1; i<4;i++){
        auto e=boost::add_edge(movingVertex, i, transitionSystem);
        transitionSystem[e.first].step=1;
    }
    boost::add_edge(1,1, transitionSystem); //trivial self-edge
    auto e2= boost::add_edge(2,2, transitionSystem); //nontrivial self-edge
    transitionSystem[e2.first].step=1;
    ts_cleanup(transitionSystem, plan);
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
    Task goal;
    if (std::get<0>(GetParam())){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    }
    configurator->init(goal);
    std::string folder=std::get<1>(GetParam());
    std::vector<vertexDescriptor> plan= get_plan(folder);
    int vertices_og=configurator->n_vertices();
    int iteration=std::get<2>(GetParam());
    for (int i=0;i<iteration-1; i++){ //simulate execution
        if (configurator->getIteration()>1){
            b2Transform deltaPose= tracker.track(configurator->getTask(), ci.data2fp, configurator->world_objects() );
            //EXPECT_FALSE(deltaPose==b2Transform_zero);
            configurator->update_graph(configurator->get_ts(), deltaPose);
        }
        configurator->change_task();
        configurator->estimate_current_vertex();    
        configurator->addIteration();
        di.newScanAvail();
        configurator->getFeatures(ci.data2fp);
        configurator->preExplore();
        EXPECT_GT(configurator->get_vertex_out_degree(0), 0);
    }
    std::vector<vertexDescriptor> updated_plan=get_plan(folder, iteration-1); //map 2
    EXPECT_EQ(di.get_iteration(), iteration);
    int vertices_now=configurator->n_vertices();
    EXPECT_LE(vertices_now, vertices_og);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);
}

INSTANTIATE_TEST_CASE_P(GoalAndMaps, HighLevelTest, ::testing::Values(
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 2),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 2),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_68cm/"), 2),
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 3),
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 4),
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 11),
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 17),
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 36),
                                                                   std::tuple<bool, std::string, int>(false, std::string("../cul_de_sac/"), 6),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 3),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 4),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 6), //,
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 17),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 38),
                                                                   std::tuple<bool, std::string, int>(true, std::string("../target_40cm/"), 89)
                                                                   ));






int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}