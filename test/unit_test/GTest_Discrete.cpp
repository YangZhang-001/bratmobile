#include "test_classes.h"
const bool DEBUG=false;


class DiscreteConfPlanTest:public DiscreteConfigurator, public testing::WithParamInterface<std::tuple<bool, std::string, int>>{

};

TEST_P(HighLevelTestDiscrete, FirstPlan){
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

class DiscreteConfiguratorTest:public DiscreteConfigurator, public testing::Test {
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

class DiscreteCTest:public DiscreteConfiguratorTest, public testing::WithParamInterface<bool>{};

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
        plan_size=1;
    }
    Spawner(); //should not replan
    if (!GetParam()){
        EXPECT_EQ(transitionSystem[m_plan[0]].direction, STOP); 
    }
    EXPECT_EQ(m_plan.size(), plan_size);
    EXPECT_EQ(simulatedTasks, simTasks);
}

class DiscreteCrashedTest: public DiscreteConfiguratorTest,public testing::WithParamInterface<float>{};

/**
 * @brief Tests whether robot finding itself in state which collided results in replanning
 */
TEST_P(DiscreteCrashedTest,EstimateInCollision) {
    setSimulationStep(.27);
    init(DebugConfigurator::generateGoalTask());
    data2fp.emplace(Pointf(0.5, 0)); //make point corresponding to obstacle
    Spawner(); //should create obstacle avoidance plan
    change_task();
    data2fp.clear();
    data2fp.emplace(Pointf(GetParam(), 0)); //just before collision
    estimate_current_vertex();
    //EXPECT_EQ(currentVertex, 3);
    Spawner(); //should not replan
    EXPECT_GT(simulatedTasks, 1);
    if (GetParam()>.15){
        EXPECT_GT(m_plan.size(), 2);
        if (m_plan.size()>0){
            vertexDescriptor plan_end=*(m_plan.end()-1);
            EXPECT_TRUE(controlGoal.checkEnded(transitionSystem[plan_end]).ended);
        }        
    }
}

class DiscretePropagateTest:public DiscreteConfiguratorTest, public testing::WithParamInterface<Direction>{};

TEST_P(DiscretePropagateTest, Propagate) {
    Disturbance d(AVOID, b2Vec2(0.5, 0));
    dummy_vertex(MOVING_VERTEX);
    auto v1=boost::add_vertex(transitionSystem);
    auto e1=boost::add_edge(currentVertex, v1, transitionSystem);
    auto v2=boost::add_vertex(transitionSystem);
    auto e2=boost::add_edge(v1, v2, transitionSystem);
    transitionSystem[v1].direction= DEFAULT;
    transitionSystem[v2].direction= GetParam();
    transitionSystem[v1].outcome= simResult::successful;
    transitionSystem[v2].outcome= simResult::crashed;
    transitionSystem[v2].Dn=d;
    std::set<vertexDescriptor>closed;
    propagateD(v2, v1);
    if (GetParam()==DEFAULT){
        EXPECT_EQ(transitionSystem[v1].outcome, simResult::safeForNow);
        EXPECT_TRUE(transitionSystem[v1].Dn.getAffIndex()==AVOID);
        EXPECT_TRUE(transitionSystem[currentVertex].Dn.getAffIndex()==AVOID);
    }
    else{
        EXPECT_NE(transitionSystem[v1].outcome, simResult::safeForNow);
        EXPECT_FALSE(transitionSystem[v1].Dn.getAffIndex()==AVOID);
        EXPECT_FALSE(transitionSystem[currentVertex].Dn.getAffIndex()==AVOID);
    }
}

INSTANTIATE_TEST_CASE_P(Directions, DiscretePropagateTest, ::testing::Values(DEFAULT, LEFT, RIGHT));

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
    GTEST_SKIP();
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
        Task t(Disturbance(), DEFAULT, b2Transform_zero, true);
        int exp=(27.0-i);
        EXPECT_EQ(int(remainingSimulationTime(&t)*10), exp);
        transitionSystem[v1].endPose=b2help::InvMul(currentTask.getAction().getTransform(MOTOR_CALLBACK), transitionSystem[v1].endPose);
    }
}
/**
 * @brief Test if robot is able to self-correct motion! (it doesnt)
 */
TEST_F(DiscreteCTest, Correct){
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
    int ts_size=transitionSystem.m_vertices.size();
    Spawner();
    change_task();
    estimate_current_vertex();
    EXPECT_NE(currentVertex, (*this_plan.begin()));
    EXPECT_TRUE(transitionSystem[currentVertex].isTurning());
    EXPECT_TRUE(currentTask.getMotorStep()==20);
    EXPECT_GT(transitionSystem.m_vertices.size(), ts_size);
}

INSTANTIATE_TEST_CASE_P(Outcomes, DiscreteCTest, ::testing::Bool());
INSTANTIATE_TEST_CASE_P(XPosition, DiscreteCrashedTest, ::testing::Values(0.16, 0.12, 0.10));
