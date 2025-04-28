#include "../test_essentials.h"

int main(int argc, char ** argv){
    WorldBuilder wb;
    BodyFeatures bf_ob;
    bf_ob.halfWidth=0.05;
    bf_ob.halfLength=0.02;
    b2Transform transform(b2Vec2(atof(argv[1]), atof(argv[2])), b2Rot(DEG_TO_RAD_K*atof(argv[3])));
    bf_ob.pose.p.x=0.5;
    bf_ob.pose.p.y=0;
    bf_ob.pose.q.Set(0);
    Disturbance goal(PURSUE, b2Vec2(1.0, 0), 0), obstacle_og(bf_ob), obstacle=obstacle_og;
    AffordanceIndex aff=AffordanceIndex(atoi(argv[5]));
    math::applyAffineTrans(transform, obstacle);
    b2Transform sensor_transform=b2Transform_zero;
    b2PolygonShape sensor= wb.sensor_box(Robot::get_vertices(), sensor_transform, &goal);
    b2AABB aabb;
    sensor.ComputeAABB(&aabb, sensor_transform, 0);
    b2Vec2 center=aabb.GetCenter(); //for debug
    Direction dir=Direction(atoi(argv[4]));
    obstacle.set_affordance(aff);
    obstacle_og.set_affordance(aff);
    Task t(obstacle, dir);
    if (aff==PURSUE && dir==DEFAULT){
        t.setEndCriteria(Distance(0.1)); //stop 10 cm before obstacle 
    }
    bool ended=t.checkEnded(sensor, b2Transform_zero , &obstacle_og);
    debug_draw(sensor.m_vertices, obstacle.vertices());
    return !ended;

}