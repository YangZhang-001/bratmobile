#include "test_classes.h"

TEST_P(HighLevelTestDiscrete, FirstPlan){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    Task goal;
    bool hasGoal=std::get<0>(GetParam()), success=false;
    if (hasGoal){
        goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
    }
    else{
        configurator->setSimulationStep(0.5);
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

INSTANTIATE_TEST_CASE_P(CulDeSac, HighLevelTestDiscrete, ::testing::Combine( ::testing::Values(false), ::testing::Values(std::string("../cul_de_sac/")), ::testing::Values(2, 3, 4, 17, 36)));
                                                                  

INSTANTIATE_TEST_CASE_P(Target40, HighLevelTestDiscrete, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_40cm/")), ::testing::Values(2, 3, 4, 6,17, 89)));

INSTANTIATE_TEST_CASE_P(Target68, HighLevelTestDiscrete, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_68cm/")), ::testing::Values(2, 3, 4, 6, 17, 36)));

class DiscreteCTest:public DiscreteConfigurator, public testing::TestWithParam<bool> {
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

    // bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v){
    //     return AttentiveConfigurator::closeVertex(closed, v);
    // }
};

TEST_F(DiscreteCTest, Replan) {
    setSimulationStep(.5);
    Spawner(); //should create obstacle avoidance plan
    int plan_size=2;
    EXPECT_EQ(m_plan.size(), plan_size);
    change_task();
    estimate_current_vertex();
    b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
    data2fp.emplace(Pointf(0.3, 0)); 
    int simTasks=15;
    plan_size++;
    Spawner(); //should replan
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks); 
    EXPECT_TRUE(currentTask.is_over());
}

TEST_P(DiscreteCTest, TestCheck) {
    setSimulationStep(.5);
    data2fp.emplace(Pointf(0.3, 0)); //make point corresponding to obstacle
    Spawner(); //should create obstacle avoidance plan
    int plan_size=3;
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


TEST_F(DiscreteCTest, DontReplan) {
    data2fp.emplace(Pointf(0.5, 0)); //make point corresponding to obstacle
    Spawner(); //should create obstacle avoidance plan
    transitionSystem[2].outcome=simResult::safeForNow;
    m_plan={2};
    int plan_size=1, simTasks=1;
    EXPECT_EQ(m_plan.size(), plan_size);
    change_task();
    estimate_current_vertex();
    b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
    Spawner(); //shouldNT replan
    plan_size--;
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks); 
    EXPECT_FALSE(currentTask.is_over());
}
/**
 * @brief checks that simulation time is calculated correctly for current DEFAULT task
 */
TEST_F(DiscreteCTest, RemainingTime){
    dummy_vertex(MOVING_VERTEX);
    vertexDescriptor v1;
    transitionSystem[DUMMY].options.push_back(DEFAULT);
    addVertex(DUMMY, v1);
    transitionSystem[v1].endPose.p.x=.27;
    //b2Transform prv=transitionSystem[v1].endPose;
    m_plan={v1};
    change_task();
    currentVertex=v1;
    iteration=2;
    for (int i=0; i<27; i++){
    b2Transform prv=transitionSystem[v1].endPose;
        Task t(Disturbance(), DEFAULT, b2Transform_zero, true);
        float exp=(27.0f-i)/10.0f;
        EXPECT_EQ(remainingSimulationTime(&t), exp);
        transitionSystem[v1].endPose=InvMul(currentTask.getAction().getTransform(MOTOR_CALLBACK), transitionSystem[v1].endPose);
    }
}

INSTANTIATE_TEST_CASE_P(Outcomes, DiscreteCTest, ::testing::Bool());

