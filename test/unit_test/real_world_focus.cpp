#include "../test_essentials.h"

int main(int argc, char** argv){
    b2Transform x;
    x.p.x=atof(argv[1]);
    x.p.y=atof(argv[2]);
    x.q.Set(DEG_TO_RAD_K *atof(argv[3]));
    BodyFeatures bf(x);
    bf.halfWidth=0.02;
    bf.halfLength=0.05;
    Disturbance dist(bf);
    dist.set_affordance(AffordanceIndex(atoi(argv[4])));
    WorldBuilder::Bridger bridge;
    Task task(dist, Direction(atoi(argv[5])), b2Transform_zero, true);
    cv::Rect2f focus=real_world_focus(&t);
    if (dist.getAffIndex()!=NONE){
        std::vector <cv::Point2f> d_vertices=dist.bf.vertices_cv();
        for (const cv::Point2f & v: d_vertices){
            if (!focus.contains(v)){
                throw std::invalid_argument("not in focus");
            }
        }
    }
    else{
        if (focus.area){
            throw std::invalid_argument("area not 0!");
        }
    }
    return 0;
}