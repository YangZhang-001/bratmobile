#include "attentive.h"
#include "publisher.h"

class TrackerGUI: public ClosedLoop_Tracker, public Publisher{
    Disturbance Di_now;
    Disturbance goal_now;
    public:
    TrackerGUI(): ClosedLoop_Tracker() {}

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
        Di_now=currentTask.get_disturbance();
        goal_now=goal.get_disturbance();
    }

};