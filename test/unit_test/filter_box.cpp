#include "../test_essentials.h"

int main(int argc, char ** argv){
    CoordinateContainer points;
    get_coordinate_container("../target_40cm/", points);
    WorldBuilder wb;
    std::pair<Pointf, Pointf> bounds, box_bounds;
    std::vector<Pointf> box_vertices;
    b2Transform start;
    start.p.x=atof(argv[1]);
    start.p.y=atof(argv[2]);
    start.q.Set(atof(argv[3]));
    float boxLength=BOX2DRANGE-ROBOT_BOX_OFFSET_X;
    Direction d=Direction(atoi(argv[4]));
    float halfWindowWidth=.15;
    bounds= wb.bounds(d, start, boxLength, halfWindowWidth);
    b2PolygonShape box= wb.object_filtering_box(halfWindowWidth, boxLength,start, d);
    b2AABB box_aabb;
    for (int i =0; i<box.m_count;i++){
        box_vertices.push_back(Pointf(box.m_vertices[i].x, box.m_vertices[i].y));
    }
    if (box_vertices.size()==0){
        throw std::invalid_argument("no vertices");
    }
    box.ComputeAABB(&box_aabb, start, 0);    
    box_bounds.first=Pointf(box_aabb.lowerBound.x, box_aabb.lowerBound.y);
    box_bounds.second=Pointf(box_aabb.upperBound.x, box_aabb.upperBound.y);
    b2Vec2 box_centroid=box.m_centroid;
    // int ct=0;
    // for (Pointf p: box_vertices){
    //     for (Pointf p2:bounds){
    //         if (fabs(p.x-p2.x)<0.01 && fabs(p.y-p2.y)<0.01){
    //             ct++;
    //         }
    //     }
    // }
    if (fabs(bounds.first.x-box_bounds.first.x)<0.02 && fabs(bounds.first.y-box_bounds.first.y)<0.02&& fabs(bounds.second.x-box_bounds.second.x)<0.02&&fabs(bounds.second.y-box_bounds.second.y)<0.02){
        return 0;
    }
    else{
        throw std::invalid_argument("whoops");
    }
    return 0;

}