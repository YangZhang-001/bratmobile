#include "attentive.h"
/**
 * Testing simulation times
 */

/**
 * @brief Each point is an object but each time the world is built, only the points in the way of the task are constructed
 */
class WorldPointBuilder: public virtual WorldBuilder{
    protected:
    std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering){
    std::vector <BodyFeatures> features;
    for (const auto & p: current){
        features.push_back(BodyFeatures(b2Transform(b2Vec2(p.x, p.y), b2Rot(0))));
    }
    return features;
    }

};

/**
 * @brief Builds all points for each task
 * 
 */
class EverythingBuilder: public virtual WorldPointBuilder{
    protected:
    void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task){
        for (const BodyFeatures & bf: world_objects){
            makeBody(w, bf);
        }
    }
};

/**
 * @brief Makes a feature for every other point and builds only those in the way of task
 * 
 */
class EveryOtherFeatureBuilder: public WorldPointBuilder{
    protected:
    std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering){
    std::vector <BodyFeatures> features;
    bool toggle=true;
    for (const auto & p: current){
        if (!toggle)continue;
        features.push_back(BodyFeatures(b2Transform(b2Vec2(p.x, p.y), b2Rot(0))));
        toggle=!toggle;
    }
    return features;
    }
};

/**
 * @brief Makes a feature for every other point and builds all points
 */
class EveryOtherPointBuilder: public virtual WorldPointBuilder, public virtual EverythingBuilder{
    protected:
    void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task){
        EverythingBuilder::buildWorld(w, start, d, disturbance, halfWindowWidth, clustering, task);
    }
};

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
    std::vector <WorldBuilder*> builders={new WorldBuilder(), new WorldPointBuilder(), new EverythingBuilder(), new EveryOtherFeatureBuilder(), new EveryOtherPointBuilder()};
    std::vector <std::string> names={"WorldBuilder", "WorldPointBuilder", "EverythingBuilder", "EveryOtherFeatureBuilder", "EveryOtherPointBuilder"};
    int ct=0;
    for (WorldBuilder *wb: builders){
        wb->set_world_objects(wb->getFeatures(data, b2Transform_zero, WorldBuilder::PARTITION));
        std::string fileName=std::string("/")+names[ct];
        Logger * logger = new Logger("WorldBuilderSpeedTest", ".", fileName.c_str(), false);
        for (float remaining=1/HZ; remaining<=10.f; remaining+=1/HZ){
            Task t;
            b2World world= b2World(GRAVITY);
            auto start = std::chrono::high_resolution_clock::now();
            wb->buildWorld(world, b2Transform_zero, DEFAULT, t.get_disturbance());
            int bodyCount=world.GetBodyCount();
            Robot robot(&world);
            t.bumping_that(world, 0, robot.body(), remaining);
            auto end = std::chrono::high_resolution_clock::now();
            logger->log("%s\t%f\t%f\t%i\t%i\n", names[ct].c_str(), remaining, std::chrono::duration<float, std::milli>(end-start).count(), wb->get_world_objects().size(), bodyCount);
        }   
    delete logger;
    std::cout<<"Tested "<<names[ct]<<std::endl;
    ct++;

    }
    //cleanup
    for (WorldBuilder *wb: builders){
        delete wb;
    }

}
