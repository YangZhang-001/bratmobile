#include "../test_essentials.h"



int main(int argc, char** argv){
    b2Transform x, B, A, A2;
    x.p.x=atof(argv[1]);
    x.p.y=atof(argv[2]);
    x.q.Set(DEG_TO_RAD_K *atof(argv[3]));
    A.p.x=atof(argv[4]);
    A.p.y=atof(argv[5]);
    A.q.Set(DEG_TO_RAD_K *atof(argv[6]));
    B=b2Mul(A, x);

    A2=math::solveAxB(x, B);
    round_mat(A2);
    if (A != A2){
        debug::print_pose(A2);
        printf("deg = %f\n", A2.q.GetAngle()*(1/DEG_TO_RAD_K));
        return 1;
    }
    return 0;
}