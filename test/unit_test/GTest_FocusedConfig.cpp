#include "test_classes.h"

class FCTest:public FocusedConfigurator, public testing::TestWithParam<bool> {
    public:
    void SetUp() override {
        register_tracker(new ClosedLoop_Tracker());
        register_planner(new HorizonStarPlanner());
        register_controller(new Wise_Controller());
    }

    void TearDown() override {
        delete tracker;
        delete planner;
        delete task_controller;
        transitionSystem=TransitionSystem(1);
    }
};

TEST_P(FCTest, TestCheck) {
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
        data2fp.emplace(Pointf(0, 0)); 
        simTasks=3;
        plan_size=0;
    }
    Spawner(); //should not replan
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks);
}

TEST_F(FCTest, Replan) {
    Spawner(); //should create obstacle avoidance plan
    int plan_size=1;
    EXPECT_EQ(m_plan.size(), plan_size);
    change_task();
    estimate_current_vertex();
    b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
    //math::MulT(dp, transitionSystem);
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
    data2fp.emplace(Pointf(0.3, 0)); 
    int simTasks=5;
    plan_size++;
    Spawner(); //should replan
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks); 
    EXPECT_TRUE(currentTask.is_over());
}

TEST_F(FCTest, DontReplan) {
    data2fp.emplace(Pointf(0.3, 0)); //make point corresponding to obstacle
    Spawner(); //should create obstacle avoidance plan
    transitionSystem[2].outcome=simResult::safeForNow;
    m_plan={2};
    int plan_size=1, simTasks=1;
    EXPECT_EQ(m_plan.size(), plan_size);
    change_task();
    estimate_current_vertex();
    b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
    //math::MulT(dp, transitionSystem);
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
    Spawner(); //should replan
    plan_size--;
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks); 
    EXPECT_FALSE(currentTask.is_over());
}

INSTANTIATE_TEST_CASE_P(Outcomes, FCTest, ::testing::Bool());

class ExecutionCheckTest: public FCTest, public testing::TestWithParam<std::tuple<b2Vec2, Direction>> {
    public:
    void SetUp() override {
        register_tracker(new ClosedLoop_Tracker());
        register_planner(new HorizonStarPlanner());
        register_controller(new Wise_Controller());
    }

    void TearDown() override {
        delete tracker;
        delete planner;
        delete task_controller;
        transitionSystem=TransitionSystem(1);
    }

};

// TEST_P(FCTest, LongRangeAvoid){

//     b2Vec2 ob_pos=std::get<0>(GetParam());
//     Direction dir=std::get<1>(GetParam());
//     std::default_random_engine generator;
//     std::normal_distribution<float> distribution(0.0,0.065); //6.5cm std dev
//     float x_noise=distribution(generator);
//     float y_noise=distribution(generator);
//     data2fp.emplace(Pointf(ob_pos.x+x_noise, ob_pos.y+y_noise)); //noisy observation
//     Disturbance d;
//     d.setAffIndex(AVOID);
//     d.setPosition(Pointf(ob_pos.x, ob_pos.y)); //noise-free prediction
//     Task t(d, dir, b2Transform_zero, true);
// }