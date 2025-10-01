#include "test_classes.h"

class FCTest:virtual public FocusedConfigurator, public testing::TestWithParam<bool> {

};

TEST_P(FCTest, TestCheck) {
    register_tracker(new ClosedLoop_Tracker());
    register_planner(new HorizonStarPlanner());
    register_controller(new Wise_Controller());
    data2fp.emplace(Pointf(0.3, 0)); //make point corresponding to obstacle
    Spawner(); //should create obstacle avoidance plan
    int plan_size=2;
    EXPECT_EQ(m_plan.size(), plan_size);
    change_task();
    estimate_current_vertex();
    b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
    math::MulT(dp, transitionSystem);
    data2fp.clear();
    data2fp.emplace(Pointf(0.3-dp.p.x, 0)); 
    int simTasks=1;
    plan_size--;
    if (!GetParam()){
        data2fp.emplace(Pointf(0, 0)); //no obstacle
        simTasks=3;
        plan_size=0;
    }
    Spawner(); //should not replan
    delete tracker;
    delete planner;
    delete task_controller;
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks); //only the first two edges should have been visited
}

INSTANTIATE_TEST_CASE_P(Outcomes, FCTest, ::testing::Bool());