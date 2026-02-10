#include "test_classes.h"
const bool DEBUG=false;


// TEST_F(HighLevelTestB2B, TrickyScenarioB2B){
//     const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
//     Logger logger=makeLogger(info);
//     configurator->register_logger(&logger);
//     configurator->init(DebugConfigurator::generateGoalTask());
//     configurator->addIteration();
//     configurator->get_worldbuilder()->add_iteration();
//     configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTricky());
//     b2World world(GRAVITY);
//     configurator->explorePlan(world);
//     EXPECT_GT(configurator->get_plan().size(), 0);
//     EXPECT_FALSE(has180Turn(configurator->get_plan()));
//     bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
//     EXPECT_TRUE(planned_to_goal);
// }


// TEST_F(HighLevelTestB2B, TrapScenarioB2B){
//     const char* info=::testing::UnitTest::GetInstance()->current_test_info()->value_param();
//     Logger logger=makeLogger(info);
//     configurator->register_logger(&logger);
//     configurator->init(DebugConfigurator::generateGoalTask());
//     configurator->addIteration();
//     configurator->get_worldbuilder()->add_iteration();
//     configurator->get_worldbuilder()->set_world_objects(CreativeWorldBuilder::makeTrickyTrap());
//     b2World world(GRAVITY);
//     configurator->explorePlan(world);
//     EXPECT_GT(configurator->get_plan().size(), 0);
//     EXPECT_FALSE(has180Turn(configurator->get_plan()));
//     bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
//     EXPECT_TRUE(planned_to_goal);
// }