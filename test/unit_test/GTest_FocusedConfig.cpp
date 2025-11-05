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

    bool has180Turn(std::vector<vertexDescriptor> plan){
    for (int i=1; i<plan.size(); i++){
        if (transitionSystem[plan[i]].isTurning() && transitionSystem[plan[i-1]].isTurning()){
            return true;
        }

    }
}
};


/**
 * Test if a) true: does not replan if not needed b) replans if needed (suddently an obstacle appears)
 */


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
        plan_size=1;
    }
    Spawner(); //should not replan
    if (!GetParam()){
        EXPECT_EQ(transitionSystem[m_plan[0]].direction, STOP); 
    }
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
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
    data2fp.emplace(Pointf(0.3, 0)); 
    int simTasks=5;
    plan_size++;
    Spawner(); //should replan
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks); 
    EXPECT_TRUE(currentTask.is_over());
}

TEST_F(FCTest, ReplanDefault) {
    setSimulationStep(.5);
    data2fp.clear();
    Spawner(); //just go straight ahead
    int plan_size=1;
    EXPECT_EQ(m_plan.size(), plan_size);
    change_task();
    estimate_current_vertex();
    b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
    data2fp.emplace(Pointf(0.8, 0)); //obstacle is far
    int simTasks=5;
    plan_size=3;
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

TEST_F(FCTest, TrickyScenario){
    init(DebugConfigurator::generateGoalTask());    addIteration();
    worldBuilder->add_iteration();
    worldBuilder->set_world_objects(CreativeWorldBuilder::makeTricky());
    b2World world(GRAVITY);
    explore_plan(world);
    EXPECT_GT(m_plan.size(), 0);
    EXPECT_TRUE(has180Turn(m_plan));
    bool planned_to_goal=controlGoal.checkEnded(transitionSystem[*(m_plan.end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);

}

/**
//  * @brief Test to see why even in a don't replan situation, more states are being created
//  */
// TEST_F(FCTest, WhyMoreStates) {
//     data2fp.emplace(Pointf(0.3, 0)); //make point corresponding to obstacle
//     Spawner(); //should create obstacle avoidance plan
//     transitionSystem[2].outcome=simResult::safeForNow;
//     m_plan={2};
//     int plan_size=1, simTasks=1;
//     EXPECT_EQ(m_plan.size(), plan_size);
//     for (int i=0; i<140; i++){
//         change_task();
//         estimate_current_vertex();
//         b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
//         update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));   
//     }
//     Spawner(); //should replan
//     plan_size--;
//     EXPECT_EQ(m_plan.size(), plan_size);
//     EXPECT_EQ(simulatedTasks, simTasks); 
//     EXPECT_FALSE(currentTask.is_over());
// }

INSTANTIATE_TEST_CASE_P(Outcomes, FCTest, ::testing::Bool());


// class ExecutionCheckTest: public FCTest, public testing::TestWithParam<std::tuple<b2Vec2, Direction>> {
//     public:
//     void SetUp() override {
//         register_tracker(new ClosedLoop_Tracker());
//         register_planner(new HorizonStarPlanner());
//         register_controller(new Wise_Controller());
//     }

//     void TearDown() override {
//         delete tracker;
//         delete planner;
//         delete task_controller;
//         transitionSystem=TransitionSystem(1);
//     }

// };
