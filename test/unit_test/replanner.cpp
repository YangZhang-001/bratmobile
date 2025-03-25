#include "../callbacks.h"

int main(int argc, char** argv){
    printf("lin 4\n");
    bool debug=1;
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
            printf("target");
        }
    }
    Task goal(target1,DEFAULT);
    Configurator conf(goal);
    conf.simulationStep=0.27;
    LIDAR_In ci;
    ControlInterface control;
    conf.registerInterface(&ci, &control);
    conf.setBenchmarking(true, "simulation_benchmarking");
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.Spawner();
    auto og_plan=conf.control->plan;
    int n_v=conf.transitionSystem.m_vertices.size();
    conf.addIteration();
    int og_step=0;
//    conf.currentVertex=control.plan[0];
    control.change_task(1, control.plan, conf.transitionSystem, conf.controlGoal, *conf.getTask(), conf.currentVertex);
   // conf.getTask()->motorStep=0;
    if (argv[1]=="empty"){
        og_plan={2};
    }    
    conf.printPlan(&conf.control->plan);
    if (!conf.control->plan.empty()){
        conf.currentVertex=*(conf.control->plan.end()-1);
        vertexDescriptor prev=*(conf.control->plan.end()-2);
        conf.currentEdge=boost::edge(prev, conf.currentVertex, conf.transitionSystem).first;

    }
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    b2Transform shift= b2Transform(b2Vec2(1,0), b2Rot(0));
    math::applyAffineTrans(shift, conf.transitionSystem);    
    if (argc>4){
        di.iteration=atoi(argv[4]);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    if (argc > 5){
        conf.controlGoal.disturbance.bf.pose = -shift;
        conf.controlGoal.disturbance.bf.pose.q.Set(M_PI);

        printf("back\n");
        og_plan={3, 5, 2};
        n_v+=7;
    }
    conf.getTask()->change=1;
    control.plan.clear();
    control.change_task(1, control.plan, conf.transitionSystem, conf.controlGoal, *conf.getTask(), conf.currentVertex);
    conf.getTask()->motorStep=100; //simulate new step setting because we are in open loop
    conf.Spawner();
    if (og_plan!=conf.control->plan){
        printf("wrong plan\n");
        return 1;
    }
    control.change_task(1, conf.control->plan, conf.transitionSystem, conf.controlGoal, *conf.getTask(), conf.currentVertex);
    conf.getTask()->motorStep=100; //simulate new step setting because we are in open loop
    conf.printPlan(&conf.control->plan);
    if (conf.transitionSystem.m_vertices.size() > n_v){
        printf("size error = %i\n", conf.transitionSystem.m_vertices.size()-n_v);
        return 2;
    }
    printf("wohoo\n");
    return 0;
}