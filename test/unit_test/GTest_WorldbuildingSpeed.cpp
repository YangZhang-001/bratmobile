#include "test_classes.h"
#include <gtest/gtest.h>
/**
 * Testing simulation times
 */

/**
 * @brief Each point is an object but each time the world is built, only the points in the way of the task are constructed
 */
class WorldPointBuilder: public WorldBuilder{

    std::vector <BodyFeatures> WorldBuilder::getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering){
    std::vector <BodyFeatures> features;
    for (const auto & p: current){
        features.push_back(BodyFeatures(b2Transform(b2Vec2(p.x, p.y), b2Rot(0))));
    }
    return features;
    }

}

/**
 * @brief Builds all points for each task
 * 
 */
class EverythingBuilder: public WorldPointBuilder{

    void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task){
        for (const BodyFeatures & bf: world_objects){
            makeBody(w, bf);
        }
    }
}

/**
 * @brief Makes a feature for every other point and builds only those in the way of task
 * 
 */
class EveryOtherFeatureBuilder: public WorldPointBuilder{

    std::vector <BodyFeatures> WorldBuilder::getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering){
    std::vector <BodyFeatures> features;
    bool toggle=true;
    for (const auto & p: current){
        if !(toggle)continue;
        features.push_back(BodyFeatures(b2Transform(b2Vec2(p.x, p.y), b2Rot(0))));
        toggle=!toggle;
    }
    return features;
    }
}

/**
 * @brief Makes a feature for every other point and builds all points
 */
class EveryOtherPointBuilder: public virtual WorldPointBuilder, public EverythingBuilder{

    void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task){
        EverythingBuilder::buildWorld(w, start, d, disturbance, halfWindowWidth, clustering, task);
    }
}


class WorldBuilderSpeedTest: public ::testing::TestWithParam<std::tuple<WorldBuilder, float>>{
    public:
    void SetUp(){    }

    void TearDown(){}

    void testSpeed(){
        Logger * logger = new Logger("WorldBuilderSpeedTest");
        WorldBuilder wb = std::get<0>(GetParam());
        Direction d = std::get<1>(GetParam());
        float remaining = std::get<2>(GetParam());
        Task t;
        wb.set_world_objects(wb.getFeatures(data2fp, b2Transform_zero, WorldBuilder::PARTITION));
        b2World world= b2World(GRAVITY);
        auto start = std::chrono::high_resolution_clock::now();
        wb.buildWorld(world, b2Transform_zero, d, t.disturbance);
        Robot robot;
        t.bumping_that(world, 0, robot.body(), remaining);
        auto end = std::chrono::high_resolution_clock::now();
        logger->log("%s\t%f\t%f\n", typeid(wb).name(), remaining, std::chrono::duration<float, std::milli>(end-start).count());
    
    }
};

class CulDeSac: public :::testing::Environment, public LIDAR_In {
  public:
    virtual void SetUp() {
        DataInterface di(this);
        di.set_folder("../cul_de_sac/");
        di.newScanAvail()

    }
    virtual void TearDown() {
      data2fp.clear(); // Code here will be called once, after all tests and test cases.
    }
};