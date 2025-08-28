#include <gtest/gtest.h>
#include "realWorldTestHeaders.h"

class TestInputConfigurator: public UserInputConfigurator{
    public:
    TestInputConfigurator(DirectionSetter *ds, AffordanceSetter *as): UserInputConfigurator(ds, as){}
    CoordinateContainer & getData2fp(){return data2fp;};
    const Disturbance & getDi(){return currentTask.get_disturbance();}
    AffordanceIndex goalAffordance(){return currentTask.get_disturbance().getAffIndex();}
    void set_world_objects(std::vector<BodyFeatures> bfs){worldBuilder.set_world_objects(bfs);};

    Task * const getGoal(){return &controlGoal;}
    Task * const getTask(){return &currentTask;}
    void setRunning (bool r){running=r;}

    void run(){
        Spawner();
		b2Transform deltaPose=b2Transform_zero;
		if (getIteration()>1){
			deltaPose= tracker->track((currentTask),ci->data2fp, worldBuilder.get_world_objects());
		}
        update_graph(transitionSystem, deltaPose);
        if (goal_changer!=NULL){
            if (( currentTask.is_over()& transitionSystem[currentVertex].direction!=STOP && m_plan.empty() && getIteration()>1)){
                goal_changer->change_goal(&controlGoal);
            }					
        }
        change_task();		
        adjust_goal_expectation();
        estimate_current_vertex();
        printf("current v=%i\n", currentVertex);
        tracker->on_new_reading(&controlGoal);
        ci->setReady(true);

    }
};

class TestTracker: public ClosedLoop_Tracker{
    public:
    b2PolygonShape getAttentionWindow(){return attention_window;}
};



class TestEnvironment: public ::testing::TestWithParam<std::tuple<AffordanceIndex, Direction>>{
    public:
    AffordanceSetter as=AffordanceSetter(NONE);
    DirectionSetter ds=DirectionSetter(UNDEFINED);
    AffordanceIndex affSolution=NONE; //what is the control goal?
    void SetUp() override {
        as= AffordanceSetter(std::get<0>(GetParam()));
        ds= DirectionSetter(std::get<1>(GetParam()));
        affSolution= std::get<0>(GetParam());
    }

    BodyFeatures makeBF(TestInputConfigurator &configurator){
        BodyFeatures bf;
        bf.halfLength=0.05;
        bf.halfWidth=0.01;
        float distance=.4; //40cm away
        if (std::get<0>(GetParam())==AVOID){
            if (std::get<1>(GetParam())==DEFAULT){
                bf.pose.p.y=distance;
            }
            else{
                bf.pose.p.x=distance;
            }
        }
        else if (std::get<0>(GetParam())==PURSUE){
            if (std::get<1>(GetParam())==DEFAULT){
                bf.pose.p.x=distance;
            }
            else if (std::get<1>(GetParam())==LEFT){
                bf.pose.p.y=distance;
                
            }
            else if (std::get<1>(GetParam())==RIGHT){
                bf.pose.p.y=-distance;
            }
        }
        configurator.set_world_objects(std::vector<BodyFeatures>{bf});
        LIDAR_In lidarIn;
        configurator.getData2fp().emplace(Pointf(bf.pose.p.x, bf.pose.p.y));
        return bf;
    }
};

TEST_P(TestEnvironment, AttentionWindow){
    TestTracker tracker;
    OneTaskController controller;
    TestInputConfigurator configurator(&ds, &as);
    BodyFeatures bf =makeBF(configurator);
    configurator.register_tracker(&tracker);
    configurator.register_controller(&controller);
    Disturbance goal(PURSUE, b2Vec2(1,0));
    configurator.init(Task(goal, UNDEFINED));
    configurator.Spawner(); //
    configurator.change_task();
    configurator.adjust_goal_expectation();
    configurator.estimate_current_vertex();
    tracker.on_new_reading(configurator.getGoal());
    EXPECT_EQ(tracker.get_tracked_disturbance()->pose().p.x,configurator.getDi().pose().p.x);
    EXPECT_EQ(tracker.get_tracked_disturbance()->pose().p.y,configurator.getDi().pose().p.y);
    EXPECT_EQ(tracker.get_tracked_disturbance()->pose().q.GetAngle(),configurator.getDi().pose().q.GetAngle());
    EXPECT_EQ(configurator.goalAffordance(), affSolution);
    if (configurator.getDi().getAffIndex()==PURSUE){
        EXPECT_TRUE(overlaps(tracker.getAttentionWindow(), tracker.get_tracked_disturbance()));
    }
    EXPECT_EQ(configurator.getDi().bf.pose.p.x,bf.pose.p.x);
    EXPECT_EQ(configurator.getDi().bf.pose.p.y,bf.pose.p.y);
  //  }
}

TEST_P(TestEnvironment, Execution){
    TestTracker tracker;
    OneTaskController controller;
    TestInputConfigurator configurator(&ds, &as);
    BodyFeatures bf =makeBF(configurator);
    configurator.register_tracker(&tracker);
    configurator.register_controller(&controller);
    Disturbance goal(PURSUE, b2Vec2(1,0));
    configurator.init(Task(goal, UNDEFINED));
    LIDAR_In lidarIn;
    Motor_Out motor;
    configurator.registerInterface(&lidarIn, &motor);
    lidarIn.data2fp=configurator.getData2fp();
    do {
        // configurator.Spawner(); //
        configurator.run();
        b2Transform newPose=InvMul(configurator.getTask()->getAction().getTransform(LIDAR_SAMPLING_RATE), bf.pose);
        lidarIn.data2fp={Pointf(newPose.p.x, newPose.p.y)};
        bf.pose=newPose;
        // configurator.change_task();
        // configurator.adjust_goal_expectation();
        // configurator.estimate_current_vertex();
    }while (configurator.getTask()->is_over());

}


INSTANTIATE_TEST_CASE_P(Inputs, TestEnvironment, ::testing::Combine(
    ::testing::Values(PURSUE, AVOID),
    ::testing::Values(DEFAULT, LEFT, RIGHT)
));