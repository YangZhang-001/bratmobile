#ifndef WORLDBUILDER_H
#define WORLDBUILDER_H
#include "sensor.h"

class WorldBuilder{
    int iteration=0;
    char bodyFile[100];
    float simulationStep=BOX2DRANGE;
    int bodies=0;

    public:
    std::vector <BodyFeatures> world_objects;
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
                                                                                                                                        //std::pair<points, obstaclestillthere>
    b2Body* makeBody(b2World&, BodyFeatures);

    std::vector <BodyFeatures> processData(const CoordinateContainer&, const b2Transform&);

    std::vector <BodyFeatures> cluster_data(const CoordinateContainer &, const b2Transform&, CLUSTERING clustering=PARTITION);

    bool checkDisturbance(Pointf, bool&,Task * curr =NULL, float range=0.025);

    std::vector <BodyFeatures> getFeatures(const CoordinateContainer &, b2Transform, CLUSTERING clustering=PARTITION);

    void buildWorld(b2World&,b2Transform, Direction,  Disturbance disturbance=Disturbance(), float halfWindowWidth=0.15, CLUSTERING clustering=CLUSTERING::PARTITION, Task * task=NULL);

    //returns top and bottom of rotated rectangle (not side-specific)
    std::pair <Pointf, Pointf> bounds(Direction, b2Transform t, float boxLength, float halfWindowWidth,std::vector <Pointf> *_bounds=NULL); //returns bottom and top of bounding box

    b2PolygonShape object_filtering_box(float halfWindowWidth, float boxLength, b2Transform start, Direction d);

    template <class Pt>
    std::pair<bool,BodyFeatures> bounding_box( std::vector <Pt >&nb){//gets bounding box of points
        float  l=(0.0005*2), w=(0.0005*2) ;
        float x_glob=0.0f, y_glob=0.0f;
        std::pair <bool, BodyFeatures> result(0, BodyFeatures());
        if (nb.empty()){
            return result;
        }
        CompareX compareX;
        CompareY compareY;
        typename std::vector<Pt>::iterator maxx=std::max_element(nb.begin(), nb.end(), compareX);
        typename std::vector<Pt>::iterator miny=std::min_element(nb.begin(), nb.end(), compareY);
        typename std::vector<Pt>::iterator minx=std::min_element(nb.begin(), nb.end(), compareX);
        typename std::vector<Pt>::iterator maxy=std::max_element(nb.begin(), nb.end(), compareY);
        if (minx->x!=maxx->x){
            w= fabs((*maxx).x-(*minx).x);
        }
        if (miny->y!=maxy->y){
            l=fabs((*maxy).y-(*miny).y);
        }
        x_glob= ((*maxx).x+(*minx).x)/2;
        y_glob= ((*maxy).y+(*miny).y)/2;
        result.second.halfLength=l/2;
        result.second.halfWidth=w/2;
        result.second.pose.p=b2Vec2(x_glob, y_glob);
        result.first=true;
        return result;
    }

    std::pair <bool, BodyFeatures> bounding_approx_poly(std::vector <cv::Point2f>nb);

    std::vector <std::vector<cv::Point2f>> kmeans_clusters( std::vector <cv::Point2f>, std::vector <cv::Point2f>&);

    std::vector <std::vector<cv::Point2f>> partition_clusters( std::vector <cv::Point2f>);

    b2Vec2 averagePoint(const CoordinateContainer &, Disturbance &, float rad = 0.025); //finds centroid of a poitn cluster, return position vec difference

    int getBodies(){
        return bodies;
    }

    int add_body_count(){
        bodies++;
    }

    void resetBodies(){
        bodies =0;
    }

    void add_iteration(int i=1){
        iteration+=1;
    }

 //   void world_cleanup(b2World &);

    b2Body * get_robot(b2World *);

    b2Fixture * get_chassis(b2Body *);

    b2AABB  makeRobotSensor(b2Body*, Disturbance *goal); //returns bounding box in world coord
    


};
#endif