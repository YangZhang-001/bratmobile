#include "test_classes.h"

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
    DebugConfigurator conf;
    conf.init(goal);
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    conf.register_controller(&wc);
    conf.register_tracker(&tracker);
    conf.setSimulationStep(.27);
    LIDAR_In ci;
    Motor_Out m;
    conf.registerInterface(&ci, &m);
    DataInterface di(&ci);
    if (argc>1){
        di.set_folder(argv[1]);
        di.newScanAvail();          
    }
    conf.set_data2fp(ci.data2fp);
    conf.addIteration();
   // b2World world(b2Vec2(0,0));
    conf.Spawner();
    int n_v=conf.n_vertices();
   // conf.printPlan(&conf.plan);
    int og=0;
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    int steps= atoi(argv[4]);
  //  int ogstep=conf.transitionSystem[conf.currentEdge].step;
    conf.getTask().getAction().setLWheelSpeed(0.5);
    conf.getTask().getAction().setRWheelSpeed(0.5);
    conf.change_task();
    int it=di.get_iteration();
    for (int i=0;i<it; i++){
        b2Transform deltaPose=conf.get_tracker()->track(conf.getTask(), ci.data2fp, conf.world_objects());
        conf.update_graph(conf.get_ts(), deltaPose);
        conf.estimate_current_vertex(conf.get_ts(), conf.getTask());
       conf.getTask().setMotorStep(conf.getTask().getMotorStep()-1);
        bool ch=conf.getTask().get_change();
        conf.change_task();
        if (ch){
            conf.getTask().setMotorStep(100); //simulate new step setting because we are in open loop
        }
        di.newScanAvail();          
        conf.set_data2fp(ci.data2fp);


        }
    // if (argc>4){
    //     di.iteration=steps;
    //     conf.addIteration(steps-conf.getIteration()+1);
    //     di.newScanAvail();          
    //     conf.data2fp = ci.data2fp;
    // }
    conf.Spawner();
   // conf.printPlan(&conf.plan);    
    int n_v_2=conf.n_vertices();
    if (n_v_2>n_v&& atoi(argv[4])<18){
        printf("difference=%i\n", n_v_2-n_v);
        return 1;
    }
    bool finished=conf.getGoal().checkEnded(conf.vertex_get_endPose(*(conf.get_plan().end()-1))).ended;
    if (finished){
        printf("plan works");
        return 0;
    }
    else{
        return 1;
    }
    return !finished;
}