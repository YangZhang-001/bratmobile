#include "../callbacks.h"

int main(int argc, char** argv){
    bool debug=0;
    Disturbance target1;
    vertexDescriptor solution=TransitionSystem::null_vertex();
    if (argc>2){
        solution=vertexDescriptor(atoi(argv[2]));
    }
    else{
        throw;
    }
    if (argc>3){
        if (atoi(argv[3])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.27;
    ClosedLoop_Tracker tracker;
    conf.register_tracker(&tracker);
    LIDAR_In ci;
    conf.registerInterface(&ci, NULL);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    conf.dummy_vertex(conf.currentVertex);
    conf.worldBuilder.world_objects=conf.worldBuilder.getFeatures(conf.data2fp, b2Transform_zero);
    conf.explorer(conf.currentVertex, conf.transitionSystem, world);
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    state_tmp.Di=conf.transitionSystem[solution].Di;
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    math::applyAffineTrans(shift, conf.transitionSystem);    
    if (argc>4){
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    std::pair<Pointf, Pointf> bt = conf.worldBuilder.bounds(DEFAULT, state_tmp.start, BOX2DRANGE, 0.15);
    std::pair <CoordinateContainer, bool> salient = conf.worldBuilder.salientPoints(state_tmp.start, conf.data2fp, bt);

    std::vector <BodyFeatures> b_features=conf.worldBuilder.getFeatures(salient.first, state_tmp.start);
    if (!b_features.empty()){
        state_tmp.Dn= Disturbance(b_features[0]); //assumes 1 item length
    }
    bool relax_match=1;
    conf.addIteration();
    boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    auto m= conf.findMatch(state_tmp,conf.transitionSystem, NULL, UNDEFINED, StateMatcher::ABSTRACT);
    if (m.second==solution){
        return 0;
    }
    else{
        StateDifference sd(conf.transitionSystem[m.second], state_tmp);
        return 1;
    }
    return 2;
}