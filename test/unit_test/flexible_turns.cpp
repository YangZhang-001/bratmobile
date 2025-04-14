#include "../test_essentials.h"

#undef DEBUG
int main(int argc, char ** argv){
    #define DEBUG 1
    Direction direction=Direction(atoi(argv[1]));
    float x=atof(argv[2]);
    float y=atof(argv[3]);
    float theta=atof(argv[4]);
    AffordanceIndex ai=AffordanceIndex(atoi(argv[5]));
    b2Transform transform(b2Vec2(x, y), b2Rot(theta));
    Disturbance disturbance(ai, transform.p, transform.q.GetAngle());
    WorldBuilder wb;
    CoordinateContainer cc;
    b2World world(b2Vec2_zero);
    if (ai==AVOID){
        cc.insert(Pointf(x,y));
    }
    wb.world_objects= wb.getFeatures(cc, b2Transform_zero);
    Task task(disturbance, direction, b2Transform_zero, true);
    wb.buildWorld(world, task.start,task.direction, task.disturbance);
    Robot robot(&world);
    simResult result= task.bumping_that(world,1, robot.body);
    if (result.resultCode==result.crashed){
        return 1;
    }
    if (direction ==LEFT && robot.body->GetTransform().q.GetAngle()<1.57){
        return 1;
    }
    if (direction ==RIGHT && robot.body->GetTransform().q.GetAngle()>-1.57){
        return 1;
    }
}