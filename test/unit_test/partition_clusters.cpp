#include "worldbuilder.h"
#include "../test_essentials.h"
int main(int argc, char**argv){
    CoordinateContainer pts;
    char filePath[256];
    sprintf(filePath, "%smap%04i.dat", argv[1], 1);
    std::ifstream file(filePath);
    float x2, y2;
    while (file>>x2>>y2){
        Pointf  p2(x2,y2);
        pts.insert(p2);
    }
    file.close();
     FILE *f;
    if (!(f=fopen(filePath, "r"))){
        throw std::invalid_argument("no file!");
    }
    else {
        fclose(f);
    }

    if (pts.size()==0){
        throw std::invalid_argument("empty vector!");
    }
    WorldBuilder wb;
    std::vector <cv::Point2f> points=set2vec2f(pts);
    std::vector<std::vector<cv::Point2f>> clusters=wb.partition_clusters(points);
    flush_points(clusters, "partition");
    return 0;


}