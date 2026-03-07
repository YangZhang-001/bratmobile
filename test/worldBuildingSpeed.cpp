#include "attentive.h"

const bool DEBUG=true;

int main(int argc, char **argv) {
  std::ifstream file("cds_test.dat");
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
    std::vector <WorldBuilder*> builders={new WorldBuilder(), new WorldPointBuilder(), new EverythingBuilder(), new EveryOtherFeatureBuilder(), new EveryOtherPointBuilder(), new LaserFocus()};
    std::vector <std::string> names={"WorldBuilder", "WorldPointBuilder", "EverythingBuilder", "EveryOtherFeatureBuilder", "EveryOtherPointBuilder", "LaserFocus"};
    int ct=0, it=1;
    for (WorldBuilder *wb: builders){
        auto start = std::chrono::high_resolution_clock::now();
        wb->set_world_objects(wb->getFeatures(data, b2Transform_zero, WorldBuilder::PARTITION));
        std::string fileName=std::string("/")+names[ct];
        Logger logger("WorldBuilderSpeedTest", ".", fileName.c_str(), false);
        auto end = std::chrono::high_resolution_clock::now();
        float buildTime=std::chrono::duration<float, std::milli>(end-start).count()/1000;
        wb->add_iteration(it);
        for (float remaining=1/HZ; remaining<=10.f; remaining+=1/HZ){
            Task t;
            b2World world(GRAVITY);
            start = std::chrono::high_resolution_clock::now();
            wb->buildWorld(world, b2Transform_zero, DEFAULT, t.get_disturbance());
            int bodyCount=world.GetBodyCount();
            Robot robot(&world);
            simResult result=t.bumping_that(world, 0, robot.body(), remaining);
            end = std::chrono::high_resolution_clock::now();
            logger.log("%f\t%i\t%i\t%i\t%f\t%i\n", std::chrono::duration<float, std::milli>(end-start).count()/1000, wb->get_world_objects().size(), bodyCount, data.size(), buildTime, result.step);
            if (result.resultCode==simResult::crashed){
                //break; //no need to simulate till it crashes
            }
            world_cleanup(world);
        }
    std::cout<<"Tested "<<names[ct]<<std::endl;
    ct++;
    it++;
    }
    //cleanup
    for (WorldBuilder *wb: builders){
        delete wb;
    }

}
