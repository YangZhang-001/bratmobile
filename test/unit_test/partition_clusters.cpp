#include "worldbuilder.h"
#include "../test_essentials.h"
int main(){
    std::vector <cv::Point2f> points, centers;
    CoordinateContainer pts;
    pts.emplace(Pointf(1.0, 0.0));
    pts.emplace(Pointf(1.1, 4.0));
    pts.emplace(Pointf(1.2, 4.3));
    pts.emplace(Pointf(1.02, 4.01));
    pts.emplace(Pointf(1.02, 4.01));

    for (Pointf p:pts){
        points.push_back(cv::Point2f(float(p.x), float(p.y)));
    }

    WorldBuilder wb;
    std::vector<std::vector<cv::Point2f>> clusters=wb.partition_clusters(points);
    flush_points(clusters, "partition");
    return 0;


}