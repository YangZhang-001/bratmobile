#include "../test_essentials.h"

int main(int argc, char** argv){
    b2Transform x, A, A2;
    x.p.x=atof(argv[1]);
    x.p.y=atof(argv[2]);
    x.q.Set(DEG_TO_RAD_K *atof(argv[3]));
    A.p.x=atof(argv[4]);
    A.p.y=atof(argv[5]);
    A.q.Set(DEG_TO_RAD_K *atof(argv[6]));
    BodyFeatures bf(x);
    bf.halfWidth=0.02;
    bf.halfLength=0.05;
    Disturbance old_d(bf), new_d=old_d;
    math::applyAffineTrans(A, new_d);
    std::vector<cv::Point2f> v_old=old_d.bf.vertices_cv(), v_new=new_d.bf.vertices_cv();
    std::vector <cv::Point2f> out(4);
    cv::Mat aff_transform=cv::estimateAffine2D(v_new, v_old,cv::noArray(), cv::LMEDS, .01, 2000, 0.999, 20);
    A2= math::transform_2d(aff_transform);
    debug_draw(v_old, "old");
    debug_draw(v_new, "new");

    round_mat(A2);
    if (A != A2){
        print_matrix(aff_transform);
        debug::print_pose(A2);
        printf("deg = %f\n", A2.q.GetAngle()*(1/DEG_TO_RAD_K));
        return 1;
    }
    return 0;
    
}