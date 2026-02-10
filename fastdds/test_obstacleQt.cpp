#include "gui.h"
#include "CppTimer.h"
#include "topics.h"
#include <fstream>
const bool DEBUG=false;

void getData(CoordinateContainer & pts){
    std::ifstream file("../test/cul_de_sac/map0001.dat");
    float x2, y2;
    while (file>>x2>>y2){
        if (b2Vec2(x2, y2).Length()<.5){
            x2 = round(x2*100)/100;
            y2 = round(y2*100)/100;
        }
        pts.emplace(Pointf(x2,y2));
    }
}

class FakeTracking:public CppTimer{
    OLTrackerGUI tracker;
    CoordinateContainer pts;
    Disturbance Dg=Disturbance(PURSUE, b2Vec2(1.0f,0.0f));
    Disturbance Di=Disturbance(AVOID, b2Vec2(.4f, .0f));
    int timerCount=0;
    void timerEvent(){
        tracker.get_transform(Task(Di, DEFAULT), pts,std::vector<BodyFeatures>());
        tracker.on_new_reading(Task(Dg, DEFAULT), Task(Di, DEFAULT));
        math::MulT(b2Transform(b2Vec2(0, 0), b2Rot(0.15)), Dg);
        math::MulT(b2Transform(b2Vec2(0.0, 0), b2Rot(0.15)), Di);
        timerCount++;
    }

    public:

    FakeTracking(){
        getData(pts);
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