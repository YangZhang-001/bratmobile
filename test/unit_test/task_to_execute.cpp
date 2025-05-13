#include "../test_essentials.h"

int main(int argc, char ** argv){
    //argv[1] = next task affordance, argv[2] = next task direction, argv[3] =goal or no goal
    Disturbance goal(PURSUE, b2Vec2(1.0, 0), 0);
    Disturbance obstacle(AVOID, b2Vec2(0.50, 0), 0);
    Task controlGoal=Task(goal, UNDEFINED);
    TransitionSystem g(1);
    g[0].endPose.p.x=0.35;
    Configurator c;
    //set goal/Di
    switch (AffordanceIndex(atoi(argv[3]))){
        case PURSUE:
        c.controlGoal=controlGoal;
        *c.getTask()=controlGoal;
        g[0].Di=goal;
        break;
        case AVOID:
        g[0].Di=obstacle;
        default:
        break;
    }
    //set Dn
    if (AffordanceIndex(atoi(argv[1]))==AVOID){
        g[0].Dn=obstacle;
    }
    //set direction
    g[0].direction=Direction(atoi(argv[2]));
    //function to test
    c.plan={0};
    Task tte= c.task_to_execute(c.plan, g, 0);
    //check that task is correct
    bool pos=tte.disturbance.pose()==g[0].Di.pose();
    bool affordance=tte.disturbance.getAffIndex()==g[0].Di.getAffIndex();
    bool dir=tte.direction==g[0].direction;
    if (g[0].direction==DEFAULT && g[0].Dn.getAffIndex()==AVOID){
        affordance=tte.disturbance.getAffIndex()==PURSUE;
        pos=tte.disturbance.pose()==g[0].Dn.pose();
    }
    if (!(pos && affordance && dir)){
        return 1;
    }  
    return 0;  

}