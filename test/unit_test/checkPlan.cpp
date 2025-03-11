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
    ConfiguratorInterface ci;
    conf.registerInterface(&ci);
    DataInterface di(&ci);
    if (argc>1){
        di.folder=argv[1];
        di.newScanAvail();          
    }
    conf.data2fp = ci.data2fp;
    conf.addIteration();
    b2World world(b2Vec2(0,0));
    // boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    // conf.dummy_vertex(conf.currentVertex);
    // conf.ci->plan_on_hold=conf.explorer(conf.currentVertex, conf.transitionSystem, world);
    // conf.ts_cleanup(&conf.transitionSystem, &(conf.ci->plan_on_hold));		

    
    // std::vector <vertexDescriptor> plan=conf.planner(conf.transitionSystem, conf.currentVertex);
    conf.Spawner();
    conf.planVertices=conf.ci->plan_on_hold;
    int n_v=conf.transitionSystem.m_vertices.size();
    conf.printPlan(&conf.planVertices);
    int og=0;
   // conf.changeTask(true, conf.ci->plan_on_hold, conf.transitionSystem);
    std::vector <vertexDescriptor> options_src;
    State state_tmp;
    int steps= atoi(argv[4]);
    //float distanceTraversed = MOTOR_CALLBACK*conf.getTask()->action.getLinearSpeed()*(steps-conf.getIteration());
    // b2Transform shift;
    // shift.q.Set(MOTOR_CALLBACK*conf.getTask()->action.getOmega()*steps);
    // shift.p.x= cos(shift.q.GetAngle())*distanceTraversed;
    // shift.p.y= sin(shift.q.GetAngle())*distanceTraversed;
    // math::applyAffineTrans(-shift, conf.transitionSystem);    
    // math::applyAffineTrans(-shift, conf.controlGoal.disturbance);
    int ogstep=conf.transitionSystem[conf.currentEdge].step;
    for (int i=0;i<di.iteration*2; i++){
        conf.trackTaskExecution(*conf.getTask());
        conf.getTask()->motorStep--;
        bool ch=conf.getTask()->change;
        conf.changeTask(conf.getTask()->change, conf.ci->plan_on_hold, conf.transitionSystem);
        if (ch){
            conf.getTask()->motorStep=100; //simulate new step setting because we are in open loop
        }
    }
    if (argc>4){
        di.iteration=steps;
        conf.addIteration(steps-conf.getIteration()+1);
        di.newScanAvail();          
        conf.data2fp = ci.data2fp;
    }
    conf.Spawner();
    conf.planVertices=conf.ci->plan_on_hold;
    conf.printPlan(&conf.planVertices);    
    int n_v_2=conf.transitionSystem.m_vertices.size();
    if (n_v_2>n_v&& atoi(argv[4])<18){
        printf("difference=%i\n", n_v_2-n_v);
        return 1;
    }
    bool finished=conf.controlGoal.checkEnded(conf.transitionSystem[*(conf.planVertices.end()-1)].endPose).ended;
    if (finished){
        printf("plan works");
        return 0;
    }
    else{
        return 1;
    }
    return !finished;
}