#include "../test_essentials.h"


int main(int argc, char** argv){
    b2Transform transform=b2Transform(b2Vec2(atof(argv[1]), atof(argv[2])), b2Rot(atof(argv[3])));
    b2Transform og=b2Transform_zero;
    if (argc>4){
    og=b2Transform(b2Vec2(atof(argv[4]), atof(argv[5])), b2Rot(atof(argv[6])));
    }
    math::applyAffineTrans(transform, og);
    debug::print_pose(og);

}