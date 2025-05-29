#include "../test_essentials.h"


int desired_split_size(b2Vec2 pos, float simulationStep){
    return int(pos.Length()/(simulationStep+0.00001))+1;
}

int main(int argc, char** argv){
    if (argc<4){
        throw std::invalid_argument("too few arguments\n");
    }

    b2Vec2 pos(atof(argv[1]), atof(argv[2]));
    b2Rot rot(atof(argv[3]));
    Task goal=Task();
    Configurator conf(goal);
    conf.simulationStep=std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH)*2;
    LIDAR_In ci;
    conf.registerInterface(&ci, NULL);
    ClosedLoop_Tracker tracker;
    conf.register_tracker(&tracker);
    b2Transform start=conf.transitionSystem[conf.movingVertex].endPose;
    if (argc>=7){
        start.p.x=atof(argv[4]);
        start.p.y=atof(argv[5]);
        start.q.Set(atof(argv[6]));
    }
    else{
        start.q.Set(rot.GetAngle());
    }
    float Dx=0.068, Dy=0, D_t=0;
    if (argc>=10){
        Dx=atof(argv[7]);
        Dy=atof(argv[8]);
        D_t=atof(argv[9]);
    }
    auto v1 = boost::add_vertex(conf.transitionSystem);
    auto e1 = boost::add_edge(conf.currentVertex, v1, conf.transitionSystem);
    conf.transitionSystem[v1].direction=DEFAULT;
    conf.transitionSystem[v1].start=start;
    conf.transitionSystem[v1].outcome=simResult::crashed;    
    conf.transitionSystem[v1].endPose=b2Transform(pos, rot);
    conf.transitionSystem[v1].Dn=Disturbance(AVOID,  b2Vec2(Dx, Dy), D_t);
    std::vector <vertexDescriptor> split =conf.splitTask(v1, conf.transitionSystem, conf.transitionSystem[v1].direction, conf.currentVertex);
    bool split_size= split.size()==desired_split_size(pos, conf.simulationStep);
    if (!split_size){
         std::cerr<<("wrong split size!");
         return -1;
    }
    int ct=0;
    for (vertexDescriptor v:split){ 
        float step_size=(conf.transitionSystem[v].endPose.p-start.p).Length();
        printf("step size=%f \t", step_size);
        if (step_size>(conf.simulationStep+0.00001)){
            std::cerr<<"wrong step size\n";
            return -1;
        }
        if (!conf.transitionSystem[v].Dn.isValid()){
             std::cerr<<("disturbance wrongly assingned\n");
        }
        if (ct<(split.size()-1) && conf.transitionSystem[v].outcome!=simResult::safeForNow){
             std::cerr<<("not setting safe for now!");
             return -1;
        }
        if (ct==(split.size()-1) && conf.transitionSystem[v].outcome!=simResult::crashed){
             std::cerr<<("not setting crashed");
             return -1;
        }
        printf("v%i: x=%f, y=%f, theta=%f\n", v, conf.transitionSystem[v].endPose.p.x, conf.transitionSystem[v].endPose.p.y, conf.transitionSystem[v].endPose.q.GetAngle());
        start=conf.transitionSystem[v].endPose;
        ct++;
    }
    return 0;
}