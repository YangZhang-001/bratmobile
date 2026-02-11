#include "test_classes.h"
const bool DEBUG=true;

class FigureTest: public HighLevelTestBase{};

/**
 * I am just running these one by one because CBA. Basically delete everything in tmp
 * and run the test you want and it'll print robot paths and lidar readings.
 * Then you can save them and rename them at this point cba
 */

/**
 * @brief Print noisy LIDAR scan and robot performance in recycling in a similar scenario
 */

TEST_F(FigureTest, PrintTransform){
    char info[20];
    CLAdaptiveTracker newTracker;
    configurator->register_tracker(&newTracker);
    b2Transform b2d_transform(b2Vec2(.4,0.05),b2Rot(-0.1f));
    sprintf(info,"%0.3f-%0.3f-%0.3f.txt",b2d_transform.p.x, b2d_transform.p.y, b2d_transform.q.GetAngle() );
    Task goal;
    b2Transform shift=b2Transform_zero;
    goal=Task(Disturbance(PURSUE, b2Vec2(1.0, 0), 0),DEFAULT);
    configurator->init(goal);
    std::string folder=std::string("../target_40cm/");
    std::vector<vertexDescriptor> plan= get_plan(folder), finished_plan;
    EXPECT_GE(configurator->get_plan().size(), 1);
    vertexDescriptor second_last_v=configurator->get_plan()[configurator->get_plan().size()-2];
    vertexDescriptor last_v=configurator->get_plan()[configurator->get_plan().size()-1];
    shift=configurator->vertex_get_endPose(last_v);
    int vertices_og=configurator->n_vertices();
    configurator->addIteration(100);
    configurator->set_current_v(last_v); //simulate plan finished
    configurator->getTask().set_change(true);
    configurator->set_plan({});
    goal=Task(Disturbance(PURSUE, b2Vec2(1.2, 0), 0),DEFAULT);
    configurator->init(goal);
<<<<<<< HEAD

=======
>>>>>>> 742cb9cab78617498bde80171a80a4c7d33a8d88
    configurator->setTask(wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(), finished_plan));
    configurator->getTask().set_change(true);
    math::MulT(shift, configurator->get_ts());
    auto points=configurator->get_data2fp();
    CoordinateContainer newPoints;
<<<<<<< HEAD
    // cv::Mat transform=(cv::Mat_<double>(2,3)<<b2d_transform.q.c,b2d_transform.q.s,b2d_transform.p.x,
    //                                         -b2d_transform.q.s,b2d_transform.q.c,b2d_transform.p.y);
    configurator->clearData();
    // cv::transform(points, newPoints, transform);
=======
    configurator->clearData();
>>>>>>> 742cb9cab78617498bde80171a80a4c7d33a8d88
    char name[50];
    sprintf(name,"/tmp/transform%s", info);
    FILE *f=fopen(name, "w+");
    CoordinateContainer cc;
    for (auto p: points){
        b2Vec2 p2d=b2Mul(b2d_transform, b2Vec2(p.x, p.y));
        fprintf(f, "%0.3f\t%0.3f\n", p2d.x, p2d.y);
        newPoints.emplace(Pointf(p2d.x, p2d.y));
    }
    fclose(f);
    configurator->set_data2fp(newPoints);
    configurator->newScanEvent();
    std::vector<vertexDescriptor> updated_plan=configurator->get_plan(); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_NEAR(vertices_now, vertices_og, 1);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);

}

// /**
//  * @brief Print checkplan perf
//  */

 
// TEST_F(HighLevelTest, CheckPlanPrint){
//     const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
//     Logger logger=makeLogger(info);
//     configurator->register_logger(&logger);
//     Task goal=Task(Disturbance(PURSUE, b2Vec2(1.0,0), 0),DEFAULT);
//     configurator->init(goal);
//     std::string folder=std::string("../target_40cm/");
//     std::vector<vertexDescriptor> plan= get_plan(folder);
//     int vertices_og=configurator->n_vertices();
//     int iteration=10;
//     trackFor(iteration);
//     std::vector<vertexDescriptor> updated_plan=get_plan(folder, iteration-1); //map 2
//     EXPECT_EQ(di.get_iteration(), iteration);
//     int vertices_now=configurator->n_vertices();
//     EXPECT_LE(vertices_now, vertices_og);
//     bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
//     bool success=planned_to_goal || configurator->getGoal().checkEnded(configurator->vertex_get_endPose(configurator->get_current_vertex())).ended;
//     EXPECT_TRUE(success);
// }

TEST_P(HighLevelInterruptTest, CheckNoisyPlan){
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
    int iteration=std::get<2>(GetParam()), taskToInterrupt=std::get<3>(GetParam());
    trackFor(iteration);
    Pointf interruptingPoint;
    std::vector<vertexDescriptor> updated_plan=get_InterruptedPlan(folder,iteration-1, taskToInterrupt, &interruptingPoint); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_GT(vertices_now, vertices_og);    
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    bool success=planned_to_goal || configurator->getGoal().checkEnded(configurator->vertex_get_endPose(configurator->get_current_vertex())).ended;
    Disturbance interruptingDisturbance(AVOID, b2Vec2(interruptingPoint.x, interruptingPoint.y), 0);
    b2World world(GRAVITY);
    configurator->get_worldbuilder()->buildWorld(world, b2Transform_zero, DEFAULT);
    Robot robot(&world); 
    if (overlaps(robot.box(), &interruptingDisturbance)){
        EXPECT_TRUE(configurator->get_plan().size()==0);
        EXPECT_FALSE(success);
    }
    else{
    }        
    EXPECT_TRUE(success);
}

INSTANTIATE_TEST_CASE_P(CulDeSacTurning, HighLevelInterruptTest, testing::Combine(::testing::Values(false), 
                                                                           ::testing::Values(std::string("../cul_de_sac/")),
                                                                           ::testing::Values(2),
                                                                           ::testing::Values(-1, 0) ));
                                                                           //synth fails, debug later!

INSTANTIATE_TEST_CASE_P(CulDeSacAvoided, HighLevelInterruptTest, testing::Combine(::testing::Values(false), 
                                                                           ::testing::Values(std::string("../cul_de_sac/")),
                                                                           ::testing::Values(30),
                                                                           ::testing::Values(-1) ));


TEST_F(HighLevelTest, TrickyScenario){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    configurator->init(DebugConfigurator::generateGoalTask());
    configurator->addIteration();
    configurator->get_worldbuilder()->add_iteration();
    configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTricky());
    b2World world(GRAVITY);
    configurator->explorePlan(world);
    EXPECT_GT(configurator->get_plan().size(), 0);
    EXPECT_TRUE(has180Turn(configurator->get_plan()));
    EXPECT_FALSE(configurator->get_plan().empty());
    if (!configurator->get_plan().empty()){
        bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
        EXPECT_TRUE(planned_to_goal);        
    }
}

TEST_F(HighLevelTest, Trapped){
    const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
    Logger logger=makeLogger(info);
    configurator->register_logger(&logger);
    configurator->init(DebugConfigurator::generateGoalTask());
    configurator->addIteration();
    configurator->get_worldbuilder()->add_iteration();
    configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTrickyTrap(.35));
    b2World world(GRAVITY);
    configurator->explorePlan(world);
    EXPECT_LE(configurator->get_plan().size(), 0);
    EXPECT_FALSE(has180Turn(configurator->get_plan()));
    EXPECT_TRUE(configurator->get_plan().empty());
    if (!configurator->get_plan().empty()){
        bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
        EXPECT_TRUE(planned_to_goal);        
    }
}