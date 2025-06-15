#include "test_classes.h"
#include <gtest/gtest.h>

TEST_F(HighLevelTest, Init){
    EXPECT_TRUE(configurator->get_motor_interface()!=(NULL));
    EXPECT_TRUE(configurator->get_lidar_interface()!= NULL);
    EXPECT_TRUE(configurator->get_tracker()!=NULL);
    EXPECT_TRUE(configurator->get_controller()!=NULL);
}


TEST_P(HighLevelTest, Plan){
    Task goal;
    if (GetParam().first){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);

    }
    init(goal);
    get_plan(GetParam().second);
    bool success=false;
    if (!GetParam().first){
        success=configurator->plan_reaches_horizon();
    }
    else{
        success=configurator->plan_reaches_goal();
    }
    configurator->printPlan();
    EXPECT_GT(configurator->get_plan().size(),0);
    EXPECT_TRUE(success);
}

TEST_P(HighLevelTest, CheckPlan){
    Task goal;
    if (GetParam().first){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);

    }
    init(goal);
    std::string folder=GetParam().second;
    get_plan(folder);
    int vertices_og=configurator->n_vertices();
    for (int i=0;i<iteration; i++){ //simulate execution
        b2Transform deltaPose= tracker.track(configurator->getTask(), ci.data2fp, configurator->world_objects() );
        configurator->update_graph(configurator->get_ts(), deltaPose);
        configurator->estimate_current_vertex(configurator->get_ts(), configurator->getTask());    

    }
    get_plan(folder, iteration); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_LE(vertices_now, vertices_og);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);
}



INSTANTIATE_TEST_CASE_P(FormPlan, HighLevelTest, ::testing::Values(std::pair(false, std::string()),
                                                                   std::pair(false, std::string("../cul_de_sac/")),
                                                                   std::pair (true, std::string("../target_40cm/")),
                                                                   std::pair (true, std::string("../target_68cm/")),
                                                                   std::pair (true, std::string("../cul_de_sac/"))));



TEST(Initialisation, InitialMap){
    DebugConfigurator configurator;
    EXPECT_EQ(configurator.n_vertices(),1);
    EXPECT_EQ(configurator.n_edges(), 0);
}

TEST_F(HighLevelTest, AcquireData){
    init();
    EXPECT_TRUE(di.has_interface());
    di.set_iteration(iteration);
    di.newScanAvail();
    EXPECT_GT(ci.data2fp.size(),0);
}

int main(int argc, char** argv){
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
    return 0;
}