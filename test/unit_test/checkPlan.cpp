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
    Wise_Controller wc;
    conf.register_controller(&wc);
    conf.simulationStep=0.27;
    LIDAR_In ci;
    Motor_Out m;
    conf.registerInterface(&ci, &m);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));
    conf.Spawner();
    int n_v=conf.transitionSystem.m_vertices.size();
    conf.printPlan(&conf.plan);
    int og=0;
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    int steps= atoi(argv[4]);
    int ogstep=conf.transitionSystem[conf.currentEdge].step;
    conf.getTask()->action.setLWheelSpeed(0.5);
    conf.getTask()->action.setRWheelSpeed(0.5);
    int it=di.iteration;
    for (int i=0;i<it; i++){
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
        b2Transform deltaPose=conf.get_tracker()->track();
        conf.update_graph(conf.transitionSystem, deltaPose);
        conf.estimate_current_vertex(conf.transitionSystem, *conf.getTask());
        conf.getTask()->motorStep--;
        bool ch=conf.getTask()->change;
        conf.change_task();
        if (ch){
            conf.getTask()->motorStep=100; //simulate new step setting because we are in open loop
        }

        }
    // if (argc>4){
    //     di.iteration=steps;
    //     conf.addIteration(steps-conf.getIteration()+1);
    //     di.newScanAvail();          
    //     conf.data2fp = ci.data2fp;
    // }
    conf.Spawner();
    conf.printPlan(&conf.plan);    
    int n_v_2=conf.transitionSystem.m_vertices.size();
    if (n_v_2>n_v&& atoi(argv[4])<18){
        printf("difference=%i\n", n_v_2-n_v);
        return 1;
    }
    bool finished=conf.controlGoal.checkEnded(conf.transitionSystem[*(conf.plan.end()-1)].endPose).ended;
    if (finished){
        printf("plan works");
        return 0;
    }
    else{
        return 1;
    }
    return !finished;
}