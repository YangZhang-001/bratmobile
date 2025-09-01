#include "test_classes.h"
#include <gtest/gtest.h>
#include "../realWorldTestHeaders.h"
class TestEnvironment;

class TestInputConfigurator: public UserInputConfigurator{
    BodyFeatures initial_bf;
    public:
    friend class TestEnvironment;
    TestInputConfigurator(){}
    TestInputConfigurator(DirectionSetter *ds, AffordanceSetter *as): UserInputConfigurator(ds, as){}
    CoordinateContainer & getData2fp(){return data2fp;};
    const Disturbance & getDi(){return currentTask.get_disturbance();}
    AffordanceIndex goalAffordance(){return currentTask.get_disturbance().getAffIndex();}
    void set_world_objects(std::vector<BodyFeatures> bfs){worldBuilder.set_world_objects(bfs);};

    Task& getGoal(){return controlGoal;}
    Task & getTask(){return currentTask;}
    void setRunning (bool r){running=r;}

    void run(){
        Spawner();
		if (getIteration()>1){
            TrackingResult trackingResult(currentTask.get_disturbance());
			trackingResult= tracker->track((currentTask),ci->data2fp, worldBuilder.get_world_objects());
            update_graph(transitionSystem, trackingResult);
		}
        if (goal_changer!=NULL){
            if (( currentTask.is_over()& transitionSystem[currentVertex].direction!=STOP && m_plan.empty() && getIteration()>1)){
                goal_changer->change_goal(&controlGoal);
            }					
        }
        change_task();	
        adjust_goal_expectation();
        estimate_current_vertex();
        printf("current v=%i\n", currentVertex);
        ci->setReady(true);
    }

    void MulPoints(b2Transform t){
        CoordinateContainer data;
        for (auto p:data2fp){
            b2Vec2 v=b2Mul(t,b2Vec2(p.x, p.y));
            data.emplace(Pointf(v.x, v.y));
        }
        data2fp=data;
    }


};

class TestInputConfiguratorFixture:public virtual TestInputConfigurator, public ::testing::Test{
    public:
    TestInputConfiguratorFixture(){}
};

class TestTracker: public ClosedLoop_Tracker{

    public:
    b2PolygonShape getAttentionWindow(){return attention_window;}

    b2AABB getAttentionAABB();

    Disturbance * get_tracked_disturbance(){
        return &tracked_disturbance;
    }

    void makeAttentionWindow(const Task &goal, const Task & currentTask){
        ClosedLoop_Tracker::makeAttentionWindow(goal, currentTask);
    }



};

b2AABB TestTracker::getAttentionAABB(){
    b2AABB aabb;
    attention_window.ComputeAABB(&aabb, b2Transform_zero, 0);
    return aabb;
}

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

    void setConfiguratorBF(TestInputConfigurator &configurator, BodyFeatures bf){
        configurator.initial_bf=bf;
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
    tracker.on_new_reading(configurator.getGoal(), configurator.getTask());
    EXPECT_EQ(tracker.get_tracked_disturbance()->pose().p.x,configurator.getDi().pose().p.x);
    EXPECT_EQ(tracker.get_tracked_disturbance()->pose().p.y,configurator.getDi().pose().p.y);
    EXPECT_EQ(tracker.get_tracked_disturbance()->pose().q.GetAngle(),configurator.getDi().pose().q.GetAngle());
    EXPECT_EQ(configurator.goalAffordance(), affSolution);
    if (std::get<0>(GetParam())==AVOID){
        EXPECT_TRUE(overlaps(tracker.getAttentionWindow(), tracker.get_tracked_disturbance()));
    }
    EXPECT_EQ(configurator.getDi().bf.pose.p.x,bf.pose.p.x);
    EXPECT_EQ(configurator.getDi().bf.pose.p.y,bf.pose.p.y);
  //  }
}



TEST_P(TestEnvironment, Execution){
    OneTaskController controller;
    TestInputConfigurator configurator(&ds, &as);
    BodyFeatures bf =makeBF(configurator);
    TestTracker tracker;
    setConfiguratorBF(configurator, bf);
    configurator.register_tracker(&tracker);
    configurator.register_controller(&controller);
    Disturbance goal(PURSUE, b2Vec2(1,0));
    configurator.init(Task(goal, UNDEFINED));
    LIDAR_In lidarIn;
    Motor_Out motor;
    configurator.registerInterface(&lidarIn, &motor);
    lidarIn.data2fp=configurator.getData2fp();
    int steps=0;
    do {
        configurator.run();
        b2Transform newPose=InvMul(configurator.getTask().getAction().getTransform(LIDAR_SAMPLING_RATE), bf.pose);
        lidarIn.data2fp={Pointf(newPose.p.x, newPose.p.y)};
        bf.pose=newPose;
        steps++;
        if (steps>50)break;
    }while (!configurator.getTask().is_over());
    EXPECT_GT(steps, 1); //should take more than one step to complete task
    EXPECT_TRUE(configurator.getTask().is_over());
}

/**
 * @brief Tests how the system adapts to noise in task execution (e.g. if the turn is not perfectly 90 degrees)
 * 
 */
TEST_F(TestInputConfiguratorFixture, NoiseTest){
    TestTracker tracker;
    Wise_Controller wc;
    Motor_Out motor;
    control=&motor;
    register_tracker(&tracker);
    register_controller(&wc);
    init(DebugConfigurator::generateGoalTask());
    data2fp= (CoordinateContainer{Pointf(0.4,0.01), Pointf(0.4, 0), Pointf(0.4,-0.01), Pointf(0.4,-0.02), Pointf(0.4,0.02)});
    EXPECT_GT(data2fp.size(), 1); //should take more than one step to complete task
    worldBuilder.set_world_objects(worldBuilder.getFeatures(data2fp, b2Transform_zero));
    EXPECT_GT(world_objects().size(),0);
    Disturbance obstacle(worldBuilder.get_world_objects()[0]);
    obstacle.validate();
    auto e1=make_successful(MOVING_VERTEX, LEFT);
    auto e2=make_successful(e1.m_target, DEFAULT);
    transitionSystem[e1.m_target].Di=obstacle;
    transitionSystem[e2.m_target].Di=obstacle;
    transitionSystem[e1.m_target].endPose.q.Set(M_PI_2);
    transitionSystem[e2.m_target].start= vertex_get_endPose(e1.m_target);
    transitionSystem[e2.m_target].endPose=b2Mul(b2Transform(b2Vec2(0.4,0), b2Rot(0)), get_ts()[e2.m_target].start);
    TrackingResult trackingResult(currentTask.get_disturbance());
    b2Transform deltaPose=b2Transform(b2Vec2(0,0), b2Rot(DEG_TO_RAD_K*10));
    //obstacle.bf.pose=b2Mul(trackingResult.displacement, obstacle.bf.pose);
    set_plan({e1.m_target, e2.m_target});
    int steps=0;
    do {
        MulPoints(trackingResult.deltaPose);
        worldBuilder.set_world_objects(worldBuilder.getFeatures(data2fp, b2Transform_zero));
        // if (iteration>1){
            trackingResult= tracker.track((currentTask),data2fp, worldBuilder.get_world_objects());
            update_graph(transitionSystem, trackingResult);
//        }
        change_task();	
        adjust_goal_expectation();
        estimate_current_vertex();
        steps++;
        iteration++;
        deltaPose=-currentTask.getAction().getTransform(LIDAR_SAMPLING_RATE);
       if (steps>50)break;
    }while (!currentTask.is_over());
    EXPECT_LT(fabs(tracker.getDeltaTransform().q.GetAngle()),M_PI_2);
    EXPECT_GT(fabs(tracker.getDeltaTransform().q.GetAngle()),0);
    EXPECT_NEAR(currentTask.from_Di().q.GetAngle(), -M_PI_2, 0.01);
    EXPECT_GT(steps, 1); //should take more than one step to complete task


}

INSTANTIATE_TEST_CASE_P(Inputs, TestEnvironment, ::testing::Combine(
    ::testing::Values(PURSUE, AVOID),
    ::testing::Values(DEFAULT, LEFT, RIGHT)
));                                                                                        //      a_ob  r_ob  a_go    r_go     Configurator::getGoalDisturbance()