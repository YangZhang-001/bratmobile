#ifndef WORLDBUILDER_H
#define WORLDBUILDER_H
#include "sensor.h"

class WorldBuilder{
    public:
    int bodies=0;
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

    int iteration=0;
    char bodyFile[100];
    float simulationStep=BOX2DRANGE;
    std::pair <CoordinateContainer, bool> salientPoints(b2Transform, const CoordinateContainer &, std::pair <Pointf, Pointf>); //gets points from the raw data that are relevant to the task based on bounding boxes
                                                                                                                                        //std::pair<points, obstaclestillthere>
    b2Body* makeBody(b2World&, BodyFeatures);

    std::vector <BodyFeatures> processData(const CoordinateContainer&, const b2Transform&);

    std::vector <BodyFeatures> cluster_data(const CoordinateContainer &, const b2Transform&, CLUSTERING clustering=PARTITION);

    bool checkDisturbance(Pointf, bool&,Task * curr =NULL, float range=0.025);

    std::vector <BodyFeatures> getFeatures(const CoordinateContainer &, b2Transform, Direction , float, float halfWindowWidth=.1, CLUSTERING clustering=BOX);

    std::vector <BodyFeatures> buildWorld(b2World&,const CoordinateContainer&, b2Transform, Direction,  Disturbance disturbance=Disturbance(), float halfWindowWidth=.15, CLUSTERING clustering=BOX, Task * task=NULL);

    std::pair <Pointf, Pointf> bounds(Direction, b2Transform t, float boxLength, float halfWindowWidth); //returns bottom and top of bounding box

    template <class Pt>
    std::pair<bool,BodyFeatures> bounding_box( std::vector <Pt >&nb){//gets bounding box of points
        float  l=(0.0005*2), w=(0.0005*2) ;
        float x_glob=0.0f, y_glob=0.0f;
        // cv::Rect2f rect(x_loc,y_loc,w, h);
        // b2Transform pose;
        std::pair <bool, BodyFeatures> result(0, BodyFeatures());
        if (nb.empty()){
            return result;
        }
        CompareX compareX;
        CompareY compareY;
        //Pointf maxx, minx, miny, maxy;
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

    void resetBodies(){
        bodies =0;
    }

    void world_cleanup(b2World *);

    b2Body * get_robot(b2World *);

    b2Fixture * get_chassis(b2Body *);

    b2AABB  makeRobotSensor(b2Body*, Disturbance *goal); //returns bounding box in world coord
    
    template <typename Pt>
    static b2PolygonShape sensor_box(const std::vector <Pt> &all_points_pt, b2Transform robot_pose, const Disturbance * dist){
        b2PolygonShape shape;
        b2Vec2 centroid(2.0, 2.0), center=centroid, center_local=b2Vec2_zero;
        float halfHeight=0, halfWidth=0;
        if (dist->isValid()){
        std::vector <b2Vec2>  d_vertices=dist->vertices(); 
        std::vector <cv::Point2f> all_points=cast_Point2f(all_points_pt);
        for (b2Vec2 p: d_vertices){
            p=b2MulT(robot_pose, p); //get local point
            all_points.push_back(cv::Point2f(p.x, p.y));
        }
        float minx=(std::min_element(all_points.begin(),all_points.end(), CompareX())).base()->x;
        float miny=(std::min_element(all_points.begin(), all_points.end(), CompareY())).base()->y;
        float maxx=(std::max_element(all_points.begin(), all_points.end(), CompareX())).base()->x;
        float maxy=(std::max_element(all_points.begin(), all_points.end(), CompareY())).base()->y;
        halfHeight=(fabs(maxy-miny))/2; //
        halfWidth=(fabs(maxx-minx))/2;
        center.x=maxx-halfWidth;
        center.y=maxy-halfHeight;
        centroid=center-center_local;  
        }
        shape.SetAsBox(halfWidth, halfHeight,centroid, 0);
        return shape;

    }


    class Bridger{
        public:
        //returns a rectangle which represents a focus of attention for finding points corresponding to input task's disturbance
        cv::Rect2f real_world_focus(const Task * );

        //calculates 2d affine transformation of input task's disturbance from t-1 to t
        /*
        /param t input task
        /param pts point cloud
        /param observed_disturbance body features of the observed disturbance
        */
        b2Transform get_transform(const Task &, const CoordinateContainer &, BodyFeatures * observed_disturbance=NULL); //returns transform between frames; option to enter a point to bodyfeatures to track Dist

        /*
        *given points, makes minimum bounding rotated box around them
        */
        std::pair <bool, BodyFeatures> bounding_rotated_box(std::vector <cv::Point2f>nb);

        //void adjust_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);                

    }wb_bridger;

};
#endif