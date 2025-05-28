#include "../test_essentials.h"

int main(int argc, char** argv){
    Task goal(Disturbance(PURSUE, b2Vec2(1.0, 0)), DEFAULT);
    float x=atof(argv[1]), y=atof(argv[2]), a=atof(argv[3]);
    Disturbance obstacle(PURSUE, b2Vec2(0.5, 0), 0);
    Configurator configurator(goal);
    Wise_Controller wc;
    wc.set_disturbance(obstacle);
    configurator.register_controller(&wc);
    vertexDescriptor v1;
    configurator.transitionSystem[0].options={DEFAULT};
    configurator.add_vertex_now(configurator.movingVertex, v1, configurator.transitionSystem, goal.disturbance, Edge(), true);
    configurator.getTask()->disturbance=obstacle;
    configurator.plan={v1};
    b2Transform deltaPose(b2Vec2(x,y), b2Rot(a));
    configurator.update_graph(configurator.transitionSystem, deltaPose);
    configurator.getTask()->disturbance.bf.pose.p.x+=0.02;
    configurator.getTask()->disturbance.bf.pose.p.y+=0.03;
    configurator.adjust_goal_expectation();
    vertexDescriptor plan_end=configurator.plan[configurator.plan.size()-1];
    b2Transform expected =b2MulT(wc.get_disturbance().pose(), configurator.transitionSystem[plan_end].Di.pose()); //position of goal wrt current disturbance
    b2Transform observed =b2MulT(configurator.getTask()->disturbance.pose(), configurator.controlGoal.disturbance.pose());
    b2Transform difference=expected-observed;
    if (difference.p.Length()>0.001 && difference.q.GetAngle()>0.001){ //accounting for rounding error
        return 1;
    }
    return 0;
}