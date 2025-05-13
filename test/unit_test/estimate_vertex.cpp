#include "../test_essentials.h"

int main(int argc, char** argv){
    Configurator c;
    if (argc<5){
        c.current_vertices={1, 2, 3};
    }
    b2Transform Di_pose(b2Vec2(0.68, 0), b2Rot(0));
    Disturbance Di(PURSUE, b2Vec2(1.0, 0), 0);
    Disturbance Dn(AVOID, Di_pose.p, Di_pose.q.GetAngle());
    TransitionSystem g(5);
    g[0].Di= Di;
    g[1].Di=Di;
    g[1].endPose.p.x+=0.27;
    g[1].Dn=Dn;
    g[2]=g[1];
    g[3]=g[1]; //belong to same task
    g[2].endPose.p.x+=0.27;
    g[2].start=g[1].endPose;
    g[3].endPose.p.x+=(0.66-g[2].endPose.p.x);
    g[3].start=g[2].endPose;
    g[4].direction=LEFT;//turn
    g[4].start=g[3].endPose;
    g[4].endPose=g[4].start;
    g[4].endPose.q.Set(M_PI_2);
    g[4].Di=Dn;
    for (int i=0; i<c.current_vertices.size();i++){
        boost::add_edge(c.current_vertices[i], c.current_vertices[i+1], g);
    }
    Wise_Controller wc;
    c.register_controller(&wc);
    Task t =wc.task_to_execute(c.current_vertices, g, 1, c.controlGoal, *c.getTask(), c.current_vertices);
    float x=0,y=0, theta=0;
    double decimal=0, ratio=0, integer=0;
    if (argc>1){
        x=atof(argv[1])*0.27;
        t.disturbance.bf.pose.p.x-=x; //simulate D getting closer
    }
    if (argc>2){
        y=atof(argv[2])*0.27; 
        t.disturbance.bf.pose.p.y-=y;       //simulate shift along y axis
    }
    if (argc>3){
        //simulate rotation
        theta=DEG_TO_RAD_K* atof(argv[3]);
        t.disturbance.bf.pose.q.Set(t.disturbance.bf.pose.q.GetAngle()+theta);
    }
    c.currentVertex=0;
    vertexDescriptor solution=c.currentVertex;
    c.estimate_current_vertex(g, t);
    decimal=std::modf(x/0.27, &integer);
    if (decimal>0.5){
        integer+=1;
    }
    try{
        solution=c.current_vertices.at(int(integer));
    }
	catch(const std::out_of_range& oor){
        try{
            solution=c.current_vertices.at(int(c.current_vertices.size()-1));
        }
        catch(const std::out_of_range& oor2){
            solution=c.movingVertex;
        }   
		printf("not in range!\n");
		//return -1;
	}
    if (solution!=c.currentVertex){
        return 1;
    }
    return 0;
    
}