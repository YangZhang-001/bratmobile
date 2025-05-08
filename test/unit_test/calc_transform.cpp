#include "../test_essentials.h"


int main(int argc, char** argv){
    b2Transform t_prev(b2Vec2(0.5, 0.02), b2Rot(0)), t_new=t_prev, result=b2Transform_zero;
    float x_inc=atof(argv[1]);
    float y_inc=atof(argv[2]);
    t_new.p.x+=x_inc;
    t_new.p.y+=y_inc;
    calc_transform(result, t_new, t_prev);
    if (x_inc>0 && result.p.x>0){
        return 1;
    }
    if (x_inc<0 && result.p.x<0){
        return 1;
    }
    if (y_inc>0 && result.p.y>0){
        return 1;
    }
    if (y_inc<0 && result.p.y<0){
        return 1;
    }
    Disturbance goal(PURSUE, b2Vec2(1, 0));
    Task t(goal, DEFAULT);
    if (y_inc!=0){
        math::applyAffineTrans(result, &t);
    }
}