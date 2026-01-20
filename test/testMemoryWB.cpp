#include "attentive.h"

int main(int argc, char **argv) {
  std::ifstream file(argv[1]);
  CoordinateContainer data;
    float x2, y2; //read data from file
        while (file>>x2>>y2){
            if (b2Vec2(x2, y2).Length()<1.0){
                x2 = round(x2*100)/100;
                y2 = round(y2*100)/100;
                Pointf  p2(x2,y2);
                data.insert(p2);
            }
        }
        file.close();
    //make vector of worldbuilders
    WorldBuilder wb;
    wb.set_world_objects(wb.getFeatures(data, b2Transform_zero, WorldBuilder::PARTITION));
}
