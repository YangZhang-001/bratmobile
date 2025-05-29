#include "../callbacks.h"



int main(int argc, char** argv){
    Configurator conf;
    b2World world(b2Vec2(0,0));
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0)), obstacle_pose=b2Transform(b2Vec2(0, -.35), b2Rot(0));

    b2Transform other_obstacle_pose; //the obstacle to be avoided while reaching a goal
    other_obstacle_pose.p.x=-obstacle_pose.p.y;
    other_obstacle_pose.p.y=obstacle_pose.p.x;     


    BodyFeatures bf(obstacle_pose), other_bf(other_obstacle_pose);
    bf.halfLength=0.02;
    bf.halfWidth=0.05;
    bf.attention=1;
    Disturbance obstacle(bf); 
    Disturbance goal(PURSUE, b2Vec2(0.5,0.5)), other_obstacle(other_bf);
    obstacle.validate();
    int expected=100;
    Task task;
    bool goal_d=atoi(argv[1]), goal_conf=atoi(argv[2]);
    conf.data2fp.emplace(getPointf(obstacle_pose.p));
    if (goal_d){
        conf.data2fp.emplace(getPointf(other_obstacle_pose.p));
    }
    conf.worldBuilder.world_objects=conf.worldBuilder.getFeatures(conf.data2fp, b2Transform_zero);
    if (!goal_d){ //just 
        task=Task(obstacle, DEFAULT, start,true);
        conf.worldBuilder.buildWorld(world, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    }
    if (goal_conf){
        Task goal_t=Task(goal, UNDEFINED);
        conf.controlGoal=goal_t;
        if (!goal_d){
            start.q.Set(M_PI_2);
            task.disturbance=other_obstacle;
            expected=19;
            conf.worldBuilder.buildWorld(world, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
        }
        else{
            task=Task(goal, DEFAULT, start,true);
            expected=abs((start.p.x-goal.bf.pose.p.x))*HZ/task.action.getLinearSpeed();
        }
    }
    Robot robot(&world);
    b2AABB aabb= conf.worldBuilder.makeRobotSensor(robot.body, &conf.controlGoal.disturbance);
    debug_draw(world, 8);
    simResult sr= task.bumping_that(world, 69, robot.body);

    //linear speed =0.1
    printf("step=%i, expected=%i\n", sr.step, expected);
    if (sr.step!=expected){
        debug::print_pose(sr.endPose);
        return 1;
    }

    return 0;
}