#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=0;
    Disturbance target1;
    if (argc>2){
        if (atoi(argv[2])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    LIDAR_In ci;
    conf.registerInterface(&ci, NULL);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    std::vector <vertexDescriptor> options_src;
    State s1, s2;
    std::pair<Pointf, Pointf> bt = conf.worldBuilder.bounds(DEFAULT, s1.start, BOX2DRANGE, 0.15);
    std::pair <CoordinateContainer, bool> salient = conf.worldBuilder.salientPoints(s1.start, conf.data2fp, bt);
    std::vector <BodyFeatures> bf1=conf.worldBuilder.getFeatures(salient.first, s1.start);
    s1.Dn= Disturbance(bf1[0]); //assumes 1 item length
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    math::applyAffineTrans(shift, s1);
    if (argc>3){
        di.iteration=atoi(argv[3]);
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    std::pair<Pointf, Pointf> bt2 = conf.worldBuilder.bounds(DEFAULT, s2.start, BOX2DRANGE, 0.15);
    std::pair <CoordinateContainer, bool> salient2 = conf.worldBuilder.salientPoints(s2.start, conf.data2fp, bt);
    std::vector <BodyFeatures> bf2=conf.worldBuilder.getFeatures(salient2.first, s2.start);
    s2.Dn= Disturbance(bf2[0]); //assumes 1 item length
    StateDifference sd(s2, s1);
    StateMatcher matcher;
    StateMatcher::StateMatch sm(sd, Threshold());
    if (!sm.Dn_exact()){
        printf("sum_d=%f\n", sd.sum_D(sd.Dn));
        return 1;
    }
    return 0;
}