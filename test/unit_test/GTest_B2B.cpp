#include "test_classes.h"

class DebugB2B: public virtual DebugConfigurator, public virtual B2BConfigurator{
    protected:
        /**
     * @brief Makes a basic expansion module,  disturbances not set. Module looks like this
     *               
     *              v2(LEFT)---v3(DEFAULT)
                   /   
                 v0 --- q1(DEFAULT)
                   \
                    v4(RIGHT) --- v5(DEFAULT)
     * 
     * Not visited by default
     */
    void make_module(vertexDescriptor mv=0);
    public:
    DebugB2B()=default;
    
    class ClearVoyanceTest:public B2BConfigurator::ClearVoyance{
        public:
            int size(){return lookaheads.size();}
    };
};

class DebugB2BTest: public DebugB2B, public testing::Test{
    protected: 
    void SetUp() override {
        iteration++;
    }
    void TearDown() override {
        transitionSystem.clear();
        transitionSystem=TransitionSystem(1);
    }


};

TEST(FrontierCrashed, predicate){
    TransitionSystem ts(2);
    ts[1].direction=DEFAULT;
    ts[1].outcome=simResult::crashed;
    ts[0].direction=LEFT;
    Frontier f(1, std::vector<vertexDescriptor>{0});
    FrontierCrashed fc(ts, LEFT);
    EXPECT_TRUE(fc(f));
}

TEST(FrontierCrashed, predicate180Turn){
    TransitionSystem ts(3);
    ts[2].direction=DEFAULT;
    ts[2].outcome=simResult::crashed;
    ts[1].direction=LEFT;
    ts[0].direction=LEFT;
    Frontier f(2, std::vector<vertexDescriptor>{0,1});
    FrontierCrashed fc(ts, LEFT);
    EXPECT_TRUE(fc(f));
}



// TEST_F(DebugB2BTest, PartiallyExplore0) {
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionMatrix(MOVING_VERTEX, DEFAULT, MOVING_VERTEX);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 0);
// }

// TEST_F(DebugB2BTest, PartiallyExplore1) {
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionSystem[3].outcome=simResult::crashed;
//     transitionMatrix(MOVING_VERTEX, DEFAULT, MOVING_VERTEX);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 1);
// }

// TEST_F(DebugB2BTest, PartiallyExplore2) {
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionSystem[3].outcome=simResult::crashed;
//     transitionSystem[5].outcome=simResult::crashed;
//     transitionMatrix(MOVING_VERTEX, DEFAULT, MOVING_VERTEX);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 2);
// }

// TEST_F(DebugB2BTest, ApplyTransitionInHindsight){
//     make_module(MOVING_VERTEX);
//     setAllVisited();
//     transitionSystem[3].outcome=simResult::crashed;
//     transitionSystem[5].outcome=simResult::crashed;
//     applyTransitionMatrix(MOVING_VERTEX, DEFAULT, false, MOVING_VERTEX, m_plan);
//     EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 2);
// }



TEST(ClearVoyance, Add){
    DebugB2BTest::ClearVoyanceTest cv;
    Disturbance d(AVOID), d2(AVOID);
    cv.add(0, d);
    EXPECT_EQ(cv.size(), 1);
    cv.add(0, d2);
    EXPECT_EQ(cv.size(), 1);
    cv.add(1, d);
    EXPECT_EQ(cv.size(), 2);
}

TEST(ClearVoyance, Query){
    DebugB2BTest::ClearVoyanceTest cv;
    Disturbance d(AVOID);
    b2Transform t=b2Transform(b2Vec2(1.0, 0), b2Rot(0));
    d.setPose(t);
    cv.add(0, d);
    EXPECT_EQ(cv.query(0).pose(), t);     
    EXPECT_EQ(cv.query(1).getAffIndex(), NONE); //not found
}

TEST_F(DebugB2BTest, AddOptionsHindSight){
    make_module(MOVING_VERTEX);
    setAllVisited();
    transitionSystem[3].outcome=simResult::crashed;
    std::vector <Direction> options={DEFAULT, LEFT, RIGHT};
    B2BConfigurator::ClearVoyance cv;
    addOptionsInHindsight(MOVING_VERTEX, 2, 3,  cv);
    EXPECT_EQ(transitionSystem[MOVING_VERTEX].options.size(), 1);
}

class HighLevelTestB2B:  public virtual HighLevelTestBase , public testing::WithParamInterface<std::tuple<bool, std::string, int>>{
    void SetUp()override{
       configurator=new DebugB2B();
       init();
    }
    
    void TearDown() override {
        HighLevelTestBase::TearDown();
    }


};

TEST_P(HighLevelTestB2B, FirstPlanB2B){
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

TEST_P(HighLevelTestB2B, CheckPlanB2B){
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

TEST_P(HighLevelTestB2B, RecycleB2B){
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
    shift=configurator->vertex_get_endPose(last_v);
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

INSTANTIATE_TEST_CASE_P(CulDeSac, HighLevelTestB2B, ::testing::Combine( ::testing::Values(false), ::testing::Values(std::string("../cul_de_sac/")), ::testing::Values(2, 3, 4, 17, 36)));
                                                                  

INSTANTIATE_TEST_CASE_P(Target40, HighLevelTestB2B, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_40cm/")), ::testing::Values(2, 3, 4, 6,17, 89)));

INSTANTIATE_TEST_CASE_P(Target68, HighLevelTestB2B, ::testing::Combine( ::testing::Values(true), ::testing::Values(std::string("../target_68cm/")), ::testing::Values(2, 3, 4, 6, 17, 36)));


void DebugB2B::make_module(vertexDescriptor mv){
    std::vector<vertexDescriptor>new_vertices;
    for (int i=0; i<5; i++){
        new_vertices.push_back(boost::add_vertex(transitionSystem));
    }
    transitionSystem[new_vertices[0]].direction=DEFAULT;
    transitionSystem[new_vertices[2]].direction=DEFAULT;
    transitionSystem[new_vertices[4]].direction=DEFAULT;
    transitionSystem[new_vertices[1]].direction=LEFT;
    transitionSystem[new_vertices[3]].direction=RIGHT;

    add_edge_withPoses(mv,new_vertices[0]);
    add_edge_withPoses(mv,new_vertices[1]);
    add_edge_withPoses(mv,new_vertices[3]);
    add_edge_withPoses(new_vertices[1],new_vertices[2]);
    add_edge_withPoses(new_vertices[3],new_vertices[4]);
}

