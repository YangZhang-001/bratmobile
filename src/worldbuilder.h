#ifndef WORLDBUILDER_H
#define WORLDBUILDER_H
#include "sensor.h"

class WorldBuilder{
    protected:
    int iteration=0;
    char bodyFile[100];
    float simulationStep=BOX2DRANGE;
    int bodies=0;
    std::vector <BodyFeatures> world_objects;    
    friend class Configurator;
    public:
    enum CLUSTERING{BOX=0, KMEANS=1, PARTITION=2}; //BOX: bounding box around points
        struct CompareCluster{
        CompareCluster()=default;

        bool operator()(const BodyFeatures & b1, const BodyFeatures & b2){ //distances of centre from start
            bool result=false;
            if (fabs(atan2(b1.pose.p.y, b1.pose.p.x))< fabs(atan2(b2.pose.p.y, b2.pose.p.x)) && b1.pose.p.Length()<= b2.pose.p.Length()){
                result=true;
            }
            return result;
        }
    };

    std::pair <CoordinateContainer, bool> salientPoints(b2Transform, const CoordinateContainer &, std::pair <Pointf, Pointf>); //gets points from the raw data that are relevant to the task based on bounding boxes
                             
    /**
     * @brief Creates a body in the box2d world, and if the features represent a disturbance to which the attention window needs to
     * be assigned, a flag is assigned to the body user data
     * 
     * @param w the box2d world
     * @param features features of the body to be created
     * @return * b2Body* 
     */
    b2Body* makeBody(b2World& w, const BodyFeatures& features);


    /**
     * @brief returns a bounding box encompassing all points provided
     * 
     * @return std::vector <BodyFeatures> 
     */
    std::vector <BodyFeatures> processData(const CoordinateContainer&, const b2Transform&);

    /**
     * @brief Cluster point cloud data using a custom algorithm
     * @param pts point cloud
     * @param start start robot transform
     * @param clustering the clustering algorithm
     * return a vector of bodyfeatures
     */
    std::vector <BodyFeatures> cluster_data(const CoordinateContainer &pts, const b2Transform& start, CLUSTERING clustering=PARTITION);

    bool checkDisturbance(Pointf, bool&,Task * curr =NULL, float range=0.025);

    /**
     * @brief clusters point clouds into objects and returns a vector of body features
     * @param current point cloud
     * @param start robot position
     * @param partition algorithm used for partition
     */
    virtual std::vector <BodyFeatures> getFeatures(const CoordinateContainer &current, b2Transform start, CLUSTERING clustering=PARTITION);

    /**
     * @brief Creates bodies (objects) in the box2d world
     * 
     * @param disturbance 
     * @param halfWindowWidth 
     * @param clustering 
     * @param task 
     */
    virtual void buildWorld(b2World&,b2Transform, Direction,  Disturbance disturbance=Disturbance(), float halfWindowWidth=0.15, CLUSTERING clustering=CLUSTERING::PARTITION, Task * task=NULL);

    //returns top and bottom of rotated rectangle (not side-specific)
    std::pair <Pointf, Pointf> bounds(Direction, b2Transform t, float boxLength, float halfWindowWidth,std::vector <Pointf> *_bounds=NULL); //returns bottom and top of bounding box

    /**
     * @brief Makes a box 
     * 
     * @param halfWindowWidth 
     * @param boxLength 
     * @param start 
     * @param d 
     * @return b2PolygonShape 
     */
    b2PolygonShape object_filtering_box(float halfWindowWidth, float boxLength, b2Transform start, Direction d);


    std::pair <bool, BodyFeatures> bounding_approx_poly(std::vector <cv::Point2f>nb);

    /**
     * @brief Clusters points using k-means algorithm
     * 
     * @return std::vector <std::vector<cv::Point2f>> 
     */
    std::vector <std::vector<cv::Point2f>> kmeans_clusters( std::vector <cv::Point2f>, std::vector <cv::Point2f>&);

    /**
     * @brief Clusters points using the partition algorithm
     * 
     * @return std::vector <std::vector<cv::Point2f>> 
     */
    std::vector <std::vector<cv::Point2f>> partition_clusters( std::vector <cv::Point2f>);

    b2Vec2 averagePoint(const CoordinateContainer &, Disturbance &, float rad = 0.025); //finds centroid of a poitn cluster, return position vec difference

    int getBodies(){
        return bodies;
    }

    void add_body_count(){
        bodies++;
    }

    void resetBodies(){
        bodies =0;
    }

    void add_iteration(int i=1){
        iteration+=i;
    }

    b2Body * get_robot(b2World *);

    b2Fixture * get_chassis(b2Body *);

    /**
     * @brief Makes the robot attention window, i.e. a distal sensor which is comprised between the extremes of the robot body and the disturbance
     *  
     * @param robotBody 
     * @param focus a disturbance representing the focus of the attention window 
     * @return b2AABB 
     */
    b2AABB  makeRobotSensor(b2Body* const robotBody, const Disturbance & focus)const; //returns bounding box in world coord
    

    std::vector <BodyFeatures>& get_world_objects(){
        return world_objects;
    }

    void set_world_objects(const std::vector <BodyFeatures>& wo){
        world_objects=wo;
    }

    void setSimulationStep(float f){simulationStep=f;}


};

/**
 * ALTERNATIVE WORLDBUILDERS
 */

/**
 * @brief Each point is an object but each time the world is built, only the points in the way of the task are constructed
 */
class WorldPointBuilder: public WorldBuilder{
    protected:
    virtual std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override;
};

/**
 * @brief Builds all points for each task 
 * 
 */
class EverythingBuilder: public virtual WorldPointBuilder{
    protected:
    virtual void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task)override;
};

/**
 * @brief Makes a feature for every other point and builds only those in the way of task
 * 
 */
class EveryOtherFeatureBuilder: public virtual WorldPointBuilder{
    protected:
    std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override;
};

/**
 * @brief Makes a feature for every other point and builds all points
 */
class EveryOtherPointBuilder: public virtual EveryOtherFeatureBuilder, public virtual EverythingBuilder{
    protected:
    std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override{
        return EveryOtherFeatureBuilder::getFeatures(current, start, clustering);
    }

    void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task)override{
        EverythingBuilder::buildWorld(w, start, d, disturbance, halfWindowWidth, clustering, task);
    }
};

/**
 * @brief Gets all points in the way of the task and makes a body which is a bounding upright box around all points
 * 
 */
class LaserFocus: public virtual WorldBuilder{ //legacy
    protected:
    CoordinateContainer m_current;
    public:
    std::vector <BodyFeatures> getFeatures(const CoordinateContainer & current, b2Transform start, CLUSTERING clustering)override;

    virtual void buildWorld(b2World & w, b2Transform start, Direction d, Disturbance disturbance, float halfWindowWidth, CLUSTERING clustering, Task * task)override;

};

#endif