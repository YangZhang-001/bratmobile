#include "test_classes.h"
const bool DEBUG=false;

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
    if (plan.empty()){
        return false;
    }
    for (int i=0; i<=plan.size(); i++){
        if (transitionSystem[plan[i+1]].isTurning() && transitionSystem[plan[i]].isTurning()){
            return true;
        }

    }
}
};

TEST_F(FCTest, getNextSrc){
    transitionSystem=TransitionSystem(4);
    std::vector<vertexDescriptor> q={2, 3};
    transitionSystem[3].options={DEFAULT};
    EXPECT_EQ(getNextSrc(q), 3);
}

/**
 * @brief Test if robot is able to self-correct motion! (it doesnt)
 */
TEST_F(FCTest, Correct){
    setSimulationStep(.5);
    running=1;
    b2Vec2 point(0.3, 0);
    data2fp.emplace(Pointf(point.x, point.y));
    Spawner();
    auto this_plan=m_plan;
    change_task();
    estimate_current_vertex();
    float angle=3*M_PI_4/2; //75 degrees
    angle=std::copysign(angle, currentTask.getAction().getOmega());
    data2fp.emplace(Pointf(point.x, point.y));
    update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(),b2Transform(b2Vec2(), b2Rot(angle))));
    currentTask.set_change(1);
    change_task();
    estimate_current_vertex();
    Spawner();
    change_task();
    estimate_current_vertex();
    EXPECT_NE(currentVertex, (*this_plan.begin()));
    EXPECT_TRUE(transitionSystem[currentVertex].isTurning());
    EXPECT_EQ(currentTask.getMotorStep(), 20);

}

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

// TEST_F(FCTest, DontReplan) {
//     data2fp.emplace(Pointf(0.3, 0)); //make point corresponding to obstacle
//     Spawner(); //should create obstacle avoidance plan
//     transitionSystem[2].outcome=simResult::safeForNow;
//     m_plan={2};
//     int plan_size=1, simTasks=1;
//     EXPECT_EQ(m_plan.size(), plan_size);
//     change_task();
//     estimate_current_vertex();
//     b2Transform dp=currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
//     //math::MulT(dp, transitionSystem);
//     update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
//     Spawner(); //shouldnt replan
//     plan_size--;
//     EXPECT_EQ(m_plan.size(), plan_size);
//     EXPECT_EQ(simulatedTasks, simTasks); 
//     EXPECT_FALSE(currentTask.is_over());
// }

TEST_F(FCTest, TrickyScenario){
    init(DebugConfigurator::generateGoalTask());    
    addIteration();
    worldBuilder->add_iteration();
    worldBuilder->set_world_objects(CreativeWorldBuilder::makeTricky());
    b2World world(GRAVITY);
    explore_plan(world);
    EXPECT_GT(m_plan.size(), 0);
    EXPECT_TRUE(has180Turn(m_plan));
    if (m_plan.empty()){
        GTEST_FAIL();
    }
    bool planned_to_goal=controlGoal.checkEnded(transitionSystem[*(m_plan.end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);

}

// TEST_F(FCTest, ReplanNoisy) {
//     init(DebugConfigurator::generateGoalTask());    
//     addIteration();
//     worldBuilder->add_iteration();
//     worldBuilder->set_world_objects(CreativeWorldBuilder::makeCulDeSac(.1, .1, b2Vec2(.4, 0)));
//     b2World world(GRAVITY);
//     explore_plan(world);
//     int plan_size=1;
//     EXPECT_EQ(m_plan.size(), plan_size);
//     change_task();
//     change_task();
//     change_task();
//     b2Transform dp(b2Vec2(0.23, 0.2), b2Rot(0.1));
//     estimate_current_vertex();
//     update_graph(transitionSystem, TrackingResult(currentTask.get_disturbance(), dp));
//     data2fp.emplace(Pointf(0.3, 0)); 
//     int simTasks=5;
//     plan_size++;
//     Spawner(); //should replan
//     EXPECT_EQ(m_plan.size(), plan_size);
//     EXPECT_EQ(simulatedTasks, simTasks); 
//     EXPECT_TRUE(currentTask.is_over());
// }

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
