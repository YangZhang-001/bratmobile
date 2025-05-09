#include "../test_essentials.h"


int main(int argc, char** argv){
    float radius=0.5;
    b2Transform t_prev(b2Vec2(radius, 0.00), b2Rot(0)), t_new=t_prev, result=b2Transform_zero, result_opp=b2Transform_zero;
    float x_inc=atof(argv[1]);
    float angle=atof(argv[2]), neg_angle=-angle;
    t_new.p.x=radius*cos(angle);
    t_new.p.y=radius*sin(angle);
    b2Transform t_opp=t_new;
    t_opp.p.x=-t_new.p.x;
    t_opp.p.y=-t_new.p.y;
    calc_transform(result, t_new, t_prev);
    result=-result;
    if (x_inc>0 && result.p.x>0){
        return 1;
    }
    if (x_inc<0 && result.p.x<0){
        return 1;
    }
    if (angle>0 && result.p.y>0){
        return 1;
    }
    if (angle<0 && result.p.y<0){
        return 1;
    }
    Disturbance goal(PURSUE, b2Vec2(1, 0));
    Task t(goal, DEFAULT);
    math::applyAffineTrans(result, &t);
    return 0;
}