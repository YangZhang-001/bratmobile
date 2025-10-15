#include "attentive.h"
// /**
//  * Testing simulation times
//  */

// /**
//  * @brief Each point is an object but each time the world is built, only the points in the way of the task are constructed
//  */
// class WorldPointBuilder: public WorldBuilder{
//     protected:
//     virtual std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override{
//     std::vector <BodyFeatures> features;
//     for (const auto & p: current){
//         features.push_back(BodyFeatures(b2Transform(b2Vec2(p.x, p.y), b2Rot(0))));
//     }
//     return features;
//     }

// };

// /**
//  * @brief Builds all points for each task 
//  * 
//  */
// class EverythingBuilder: public virtual WorldPointBuilder{
//     protected:
//     virtual void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task)override{
//         for (const BodyFeatures & bf: world_objects){
//             makeBody(w, bf);
//         }
//     }
// };

// /**
//  * @brief Makes a feature for every other point and builds only those in the way of task
//  * 
//  */
// class EveryOtherFeatureBuilder: public virtual WorldPointBuilder{
//     protected:
//     std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override{
//     std::vector <BodyFeatures> features;
//     bool toggle=true;
//     for (const auto & p: current){
//         if (toggle){
//             features.push_back(BodyFeatures(b2Transform(b2Vec2(p.x, p.y), b2Rot(0))));
//         }
//         toggle=!toggle;
//     }
//     return features;
//     }
// };

// /**
//  * @brief Makes a feature for every other point and builds all points
//  */
// class EveryOtherPointBuilder: public virtual EveryOtherFeatureBuilder, public virtual EverythingBuilder{
//     protected:
//     std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override{
//         return EveryOtherFeatureBuilder::getFeatures(current, start, clustering);
//     }

//     void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task)override{
//         EverythingBuilder::buildWorld(w, start, d, disturbance, halfWindowWidth, clustering, task);
//     }
// };

// /**
//  * @brief Gets all points in the way of the task and makes a body which is a bounding upright box around all points
//  * 
//  */
// class LaserFocus: public virtual WorldBuilder{ //legacy
//     protected:
//     CoordinateContainer m_current;
//     public:
//     std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override{
//         m_current=current;
//         std::vector <BodyFeatures> features;
//         return features; //no features
//     }

//     virtual void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task)override{
//         std::vector <BodyFeatures> features;
//         std::pair<Pointf, Pointf> bt = bounds(d, start, simulationStep, halfWindowWidth);
//         std::pair <CoordinateContainer, bool> salient = salientPoints(start,m_current, bt);
//         if (salient.first.empty()){
//             return;
//         }
//         if (clustering==BOX){
//             features =processData(salient.first, start);
//         }
//         else{
//             features=cluster_data(salient.first, start,clustering);
//         }
//         for (const BodyFeatures & bf: features){
//             makeBody(w, bf);
//         }
//     }

// };

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
    int ct=0;
    for (WorldBuilder *wb: builders){
        auto start = std::chrono::high_resolution_clock::now();
        wb->set_world_objects(wb->getFeatures(data, b2Transform_zero, WorldBuilder::PARTITION));
        std::string fileName=std::string("/")+names[ct];
        Logger logger("WorldBuilderSpeedTest", ".", fileName.c_str(), false);
        auto end = std::chrono::high_resolution_clock::now();
        float buildTime=std::chrono::duration<float, std::milli>(end-start).count()/1000;
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
    }
    //cleanup
    for (WorldBuilder *wb: builders){
        delete wb;
    }

}
