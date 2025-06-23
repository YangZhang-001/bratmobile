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
    bool hasGoal=GetParam().first, success=false;
    if (hasGoal){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);

    }
    configurator->init(goal);
    std::string folder=GetParam().second;
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
    if (GetParam().first){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    }
    configurator->init(goal);
    std::string folder=GetParam().second;
    get_plan(folder);
    int vertices_og=configurator->n_vertices();
    int iteration=2;
    DeadReckoner deadReckoner;
    for (int i=0;i<iteration; i++){ //simulate execution
        b2Transform deltaPose= deadReckoner.track(configurator->getTask(), ci.data2fp, configurator->world_objects() );
        configurator->update_graph(configurator->get_ts(), deltaPose);
        configurator->estimate_current_vertex(configurator->get_ts(), configurator->getTask());    
        configurator->change_task();
    }
    get_plan(folder, iteration); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_LE(vertices_now, vertices_og);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);
}



INSTANTIATE_TEST_CASE_P(GoalAndMaps, HighLevelTest, ::testing::Values(
                                                                   std::pair<bool, std::string>(false, std::string("../cul_de_sac/")),
                                                                   std::pair<bool, std::string>(true, std::string("../target_40cm/"))));

INSTANTIATE_TEST_CASE_P(Obstacle68, HighLevelTest, ::testing::Values(
                                                                   std::pair<bool, std::string>(true, std::string("../target_68cm/"))));







int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}