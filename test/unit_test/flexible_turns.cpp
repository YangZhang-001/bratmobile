#include "../test_essentials.h"

#undef DEBUG
#define DEBUG true
int main(int argc, char ** argv){
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
    cc.insert(Pointf(x,y));
    wb.getFeatures(cc, b2Transform_zero);
    Task task(disturbance, direction, b2Transform_zero, true);
    wb.buildWorld(world, task.start,task.direction);
    Robot robot(&world);
    simResult result= task.bumping_that(world,1, robot.body);

}