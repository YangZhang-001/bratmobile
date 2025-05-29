#include "../callbacks.h"



int main(int argc, char** argv){
    Configurator conf;
    b2World world(b2Vec2(0,0));
    //boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    b2Transform start=b2Transform(b2Vec2(0,0), b2Rot(0)), obstacle_pose=b2Transform(b2Vec2(0, -.35), b2Rot(0));

    b2Transform other_obstacle_pose; //the obstacle to be avoided while reaching a goal
    other_obstacle_pose.p.x=-obstacle_pose.p.y;
    other_obstacle_pose.p.y=obstacle_pose.p.x;     


    BodyFeatures bf(obstacle_pose);
    bf.halfLength=0.02;
    bf.halfWidth=0.05;
    bf.attention=1;
    BodyFeatures other_bf=bf;
    other_bf.pose=other_obstacle_pose;
    Disturbance obstacle(bf); 
    Disturbance goal(PURSUE, b2Vec2(0.5,0.5)), other_obstacle(other_bf);
    obstacle.validate();
    other_obstacle.validate();
    int expected=100;
    Task task;
    bool is_goal_Di=atoi(argv[1]), goal_conf=atoi(argv[2]);
    conf.data2fp.emplace(getPointf(obstacle_pose.p));
    Robot robot(&world);
    if (!is_goal_Di){
        conf.data2fp.emplace(getPointf(other_obstacle_pose.p));
    }
    conf.worldBuilder.world_objects=conf.worldBuilder.getFeatures(conf.data2fp, b2Transform_zero);
    if (!is_goal_Di && !goal_conf){ //just avoiding an obstacle
        task=Task(obstacle, DEFAULT, start,true);
        //conf.worldBuilder.buildWorld(world, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
    }
    if (goal_conf){ //the goal is to reach a target
        Task goal_t=Task(goal, UNDEFINED);
        conf.controlGoal=goal_t;
        if (!is_goal_Di){ //the goal is not the Di of this task
            start.q.Set(M_PI_2);
            robot.body->SetTransform(start.p, start.q.GetAngle());
            task=Task(other_obstacle, DEFAULT, start, true);
            expected=21;
           // conf.worldBuilder.buildWorld(world, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);
        }
        else{ //the goal is to reach the target in this task
            task=Task(goal, DEFAULT, start,true);
            expected=abs((start.p.x-goal.bf.pose.p.x))*HZ/task.action.getLinearSpeed();
        }
    }
    b2AABB aabb= conf.worldBuilder.makeRobotSensor(robot.body, &conf.controlGoal.disturbance);
    conf.worldBuilder.buildWorld(world, task.start, task.direction, task.disturbance,0.15, WorldBuilder::PARTITION);   
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