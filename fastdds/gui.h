#ifndef GUI_H
#define GUI_H
#include "tracker.h"
#include "publisher.h"
#include "topics.h"

class TrackerGUI{
    protected:
    ObjectPackagePublisher DiPub, goalPub, attentionPub;
    public:

    ObjectPackage makeObjectPackage(const std::vector<b2Vec2> &vertices){
        ObjectPackage object;
        object.v1_x(vertices[0].x); //tr
        object.v1_y(vertices[0].y);
        object.v2_x(vertices[1].x); //br
        object.v2_y(vertices[1].y);
        object.v3_x(vertices[2].x); //bl
        object.v3_y(vertices[2].y);
        object.v4_x(vertices[3].x); //tl
        object.v4_y(vertices[3].y);
        return object;
    }

};

/**
* @brief Publishes Di/Dg to Qt Window for debugging
*/
class OLTrackerGUI: public DeadReckoner, public TrackerGUI{
    ObjectPackagePublisher DiPub, goalPub, attentionPub;

    public:
    OLTrackerGUI(): DeadReckoner() {
        if(!DiPub.init(Di_topic)){std::cerr << "Could not init the Di subscriber." << std::endl;}        
        if(!goalPub.init(Goal_topic)){std::cerr << "Could not init the goal subscriber." << std::endl;}        

    }


    void on_new_reading(const Task & goal, const Task &currentTask)override{
        DeadReckoner::on_new_reading(goal, currentTask);
        ObjectPackage DiPack=makeObjectPackage(currentTask.get_disturbance().bodyFeatures().vertices());
        ObjectPackage GoalPack=makeObjectPackage(goal.get_disturbance().bodyFeatures().vertices());
        if(!DiPub.publish(DiPack)) {std::cout<<"did not publish Di\n";}
        if(!goalPub.publish(GoalPack)) {std::cout<<"did not publish goal\n";}
    }


};

/**
* @brief Publishes Di/Dg/attention window to Qt window
 */

class CLTrackerGUI: public ClosedLoop_Tracker, public TrackerGUI{
    ObjectPackagePublisher DiPub, goalPub, attentionPub;

    public:
    CLTrackerGUI(): ClosedLoop_Tracker() {
        if(!DiPub.init(Di_topic)){std::cerr << "Could not init the Di subscriber." << std::endl;}        
        if(!goalPub.init(Goal_topic)){std::cerr << "Could not init the goal subscriber." << std::endl;}        
        if(!attentionPub.init(attention_topic)){std::cerr << "Could not init the attention subscriber." << std::endl;}        

    }

    std::vector<b2Vec2> attentionWindowVertices(){
        std::vector <b2Vec2> result;
        for (int i=0; i<attention_window.m_count; i++){
            result.push_back(*(attention_window.m_vertices+i));
        }
    return result;
    }



    void on_new_reading(const Task & goal, const Task &currentTask)override{
        ClosedLoop_Tracker::on_new_reading(goal, currentTask);
        ObjectPackage DiPack=makeObjectPackage(currentTask.get_disturbance().bodyFeatures().vertices());
        ObjectPackage GoalPack=makeObjectPackage(goal.get_disturbance().bodyFeatures().vertices());
        ObjectPackage attentionPack=makeObjectPackage(attentionWindowVertices());
        if(!DiPub.publish(DiPack)) {std::cout<<"did not publish Di\n";}
        if(!goalPub.publish(GoalPack)) {std::cout<<"did not publish goal\n";}
        if (!attentionPub.publish(attentionPack)){std::cout<<"did not publish attention\n";}
    }


};
#endif