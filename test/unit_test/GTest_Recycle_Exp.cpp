#include "test_classes.h"
const bool DEBUG=false;

TEST_P(CLAdaptiveTrackerTest, Threshold){
    State s;
    s.endPose.p.x=GetParam();
    Threshold _threshold=get_threshold(s);
    EXPECT_NEAR(_threshold.for_robot_position(), GetParam()/2, 0.01);
}

INSTANTIATE_TEST_CASE_P(Ends, CLAdaptiveTrackerTest, testing::Values(0.5, 0.05));

TEST_P(RecycleTest, Transform){
    // char info[20];
    CLAdaptiveTracker newTracker;
    configurator->register_tracker(&newTracker);
    b2Transform b2d_transform(b2Vec2(std::get<0>(GetParam()),std::get<1>(GetParam())), b2Rot(std::get<2>(GetParam())));
    // sprintf(info,"%0.3f-%0.3f-%0.3f.txt",b2d_transform.p.x, b2d_transform.p.y, b2d_transform.q.GetAngle() );
    Logger logger("RecycleTests", "\tmp", "recycle");
    configurator->register_logger(&logger);
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
    configurator->setTask(wc.next_task(configurator->getTask(), configurator->getGoal(), configurator->get_ts(), configurator->get_current_vertices(), finished_plan));
    configurator->getTask().set_change(true);
    math::MulT(shift, configurator->get_ts());
    auto points=configurator->get_data2fp();
    CoordinateContainer newPoints;
    // cv::Mat transform=(cv::Mat_<double>(2,3)<<b2d_transform.q.c,b2d_transform.q.s,b2d_transform.p.x,
    //                                         -b2d_transform.q.s,b2d_transform.q.c,b2d_transform.p.y);
    configurator->clearData();
    // cv::transform(points, newPoints, transform);
    // char name[50];
    // sprintf(name,"/tmp/transform%s", info);
    // FILE *f=fopen(name, "w+");
    // CoordinateContainer cc;
    // for (auto p: points){
    //     b2Vec2 p2d=b2Mul(b2d_transform, b2Vec2(p.x, p.y));
    //     fprintf(f, "%0.3f\t%0.3f\n", p2d.x, p2d.y);
    //     newPoints.emplace(Pointf(p2d.x, p2d.y));
    // }
    // fclose(f);
    configurator->set_data2fp(newPoints);
    configurator->newScanEvent();
    std::vector<vertexDescriptor> updated_plan=configurator->get_plan(); //map 2
    int vertices_now=configurator->n_vertices();
    EXPECT_NEAR(vertices_now, vertices_og, 1);
    bool planned_to_goal=configurator->getGoal().checkEnded(configurator->get_ts()[*(configurator->get_plan().end()-1)].endPose).ended;
    EXPECT_TRUE(planned_to_goal);

}

INSTANTIATE_TEST_CASE_P(Transforms2D, RecycleTest, ::testing::Combine(testing::Range(-.2f, .4f, 0.01f),
                                                                    ::testing::Range(-0.05f, 0.05f, 0.01f),
                                                                    ::testing::Values(-0.1f,0.1f, 0.1f)));

