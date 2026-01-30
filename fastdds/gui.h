#ifndef GUI_H
#define GUI_H
#include "tracker.h"
#include "publisher.h"
#include "topics.h"
//for enabling logs
#include <pwd.h>
#include <unistd.h>
// #include <sys/param.h>

//export FASTDDS_ENVIRONMENT_FILE=fastdds.xml


// class RobotPublisher:public ObjectPackagePublisher{
//     // public:

//     // void publish(ObjectPackage object){
//     //     if (publish(object)){
//     //     } else {
//     //         std::cout << "No messages sent as there is no listener." << std::endl;
//     //     }
    
//     // }
// };

class TrackerGUI: public ClosedLoop_Tracker{
    ObjectPackagePublisher DiPub, goalPub, attentionPub;

    public:
    TrackerGUI(): ClosedLoop_Tracker() {
        if(!DiPub.init(Di_topic)){std::cerr << "Could not init the Di subscriber." << std::endl;}        
        if(!goalPub.init(Goal_topic)){std::cerr << "Could not init the goal subscriber." << std::endl;}        
        if(!attentionPub.init(attention_topic)){std::cerr << "Could not init the attention subscriber." << std::endl;}        

    }


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

    std::vector<b2Vec2> attentionWindowVertices(){
        std::vector <b2Vec2> result;
        for (int i=0; i<attention_window.m_count; i++){
            result.push_back(*(attention_window.m_vertices+i));
        }
    return result;
    }

    // std::vector <b2Vec2> DiVertices(){
    //     return Di_now.vertices();
    // }

    // std::vector<b2Vec2> goalVertices(){
    //     return goal_now.vertices();
    // }


    void on_new_reading(const Task & goal, const Task &currentTask)override{
        ClosedLoop_Tracker::on_new_reading(goal, currentTask);
        ObjectPackage DiPack=makeObjectPackage(currentTask.get_disturbance().bodyFeatures().vertices());
        ObjectPackage GoalPack=makeObjectPackage(goal.get_disturbance().bodyFeatures().vertices());
        ObjectPackage attentionPack=makeObjectPackage(attentionWindowVertices());
        if(!DiPub.publish(DiPack)) {std::cout<<"did not publish Di\n";}
        if(!goalPub.publish(GoalPack)) {std::cout<<"did not publish goal\n";}
        if (!attentionPub.publish(attentionPack)){std::cout<<"did not publish attention\n";}
        DiPub.printTopics();
    }


};
#endif