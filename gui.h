#include "attentive.h"
#include "publisher.h"
#include "window.h"

class RobotPublisher:public ObjectPackagePublisher{
    public:
    void setTopic(std::string str){
        topic_ = participant_->create_topic(str, "ObjectPackage", TOPIC_QOS_DEFAULT);

    }

    void publish(ObjectPackage object){
        if (mypub.publish(object)){
        } else {
            std::cout << "No messages sent as there is no listener." << std::endl;
        }
    
    }
};

class TrackerGUI: public ClosedLoop_Tracker{
    RobotPublisher DiPub, goalPub, attentionPub;

    public:
    TrackerGUI(): ClosedLoop_Tracker() {
        assignTopics();
    }

    void assignTopics(){
        DiPub.setTopic(Di_topic);
        goalPub.setTopic(Goal_topic);
        attentionPub.setTopic(attention_topic);
    }

    ObjectPackage makeObjectPackage(const std::vector<b2Vec2> &vertices){
        ObjectPackage op;
        object.v1_x(vertices[0].x); //tr
        object.v1_y(vertices[0].y);
        object.v2_x(vertices[1].x); //br
        object.v2_y(vertices[1].y);
        object.v3_x(vertices[2].x); //bl
        object.v3_y(vertices[2].y);
        object.v4_x(vertices[3].x); //tl
        object.v4_y(vertices[3].y);

    }

    std::vector<b2Vec2> attentionWindowVertices(){
        std::vector <b2Vec2> result;
        for (int i=0; i<attention_window.m_count; i++){
            result.push_back(*(attention_window.m_vertices+i));
        }
    return result;
    }

    std::vector <b2Vec2> DiVertices(){
        return Di_now.vertices();
    }

    std::vector<b2Vec2> goalVertices(){
        return goal_now.vertices();
    }

    void on_new_reading(const Task & goal, const Task &currentTask)override{
        ClosedLoop_Tracker::on_new_reading(goal, currentTask);
        ObjectPackage DiPack=makeObjectPackage(currentTask.get_disturbance().bodyFeatures().vertices());
        ObjectPackage GoalPack=makeObjectPackage(goal.get_disturbance().bodyFeatures().vertices());
        ObjectPackage attentionPack=makeObjectPackage(attentionWindowVertices());
        DiPub.publish(DiPack);
        goalPub.publish(GoalPack);
        attentionPub.publish(attentionPack);
    }


};