#include "gui.h"
#include "CppTimer.h"
#include "topics.h"
const bool DEBUG=false;
class FakeTracking:public CppTimer{
    CLTrackerGUI tracker;
    Disturbance Dg=Disturbance(PURSUE, b2Vec2(1.0f,0.0f));
    Disturbance Di=Disturbance(AVOID, b2Vec2(.4f, .0f));
    int timerCount=0;
    void timerEvent(){
        tracker.on_new_reading(Task(Dg, DEFAULT), Task(Di, DEFAULT));
        math::MulT(b2Transform(b2Vec2(0, 0), b2Rot(0.15)), Dg);
        math::MulT(b2Transform(b2Vec2(0.0, 0), b2Rot(0.15)), Di);
        timerCount++;
        if (timerCount==10){ 
            tracker.init(Task());
        }
        else if (timerCount==20){
            tracker.init(Task(Dg, DEFAULT));
            timerCount=0;
        }

    }

    public:

    FakeTracking(){
        tracker.init(Task(Dg, DEFAULT));
        Di.bf.halfLength=.1;
        Di.bf.halfWidth=.02;
        Dg.bf.halfLength=.05;
        Dg.bf.halfWidth=.05;
    }
};

int main(){
    FakeTracking fakeTracking;
    fakeTracking.startms(100);
    while (!getchar()){}
    fakeTracking.stop();
}