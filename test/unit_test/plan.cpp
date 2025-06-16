#include "../callbacks.h"
std::vector <Direction> getPlan(const TransitionSystem & g, const std::vector <vertexDescriptor>& plan, vertexDescriptor pre){
    	std::vector <Direction>result;
        for (vertexDescriptor v: plan){
		std::pair <edgeDescriptor, bool> edge=boost::edge(pre, v, g);
		if (edge.second){
			// auto a=dirmap.find(g[edge.first].direction);
			// printf("%i, %s, ", edge.first.m_target, (*a).second);
            result.push_back(g[v].direction);
		}
		pre=edge.first.m_target;
		}
        return result;

}

class DebugConfigurator:public AttentiveConfigurator{
    public:
    friend class HighLevelTest;
    int n_edges(){return transitionSystem.m_edges.size();}

    int n_vertices(){return transitionSystem.m_vertices.size();}

    const std::vector <vertexDescriptor>& get_plan(){ return plan;}

    bool plan_reaches_horizon();

    bool plan_reaches_goal();

    vertexDescriptor plan_end(){return plan[plan.size()-1];}

    b2Vec2 plan_end_b2Vec2(){return transitionSystem[plan_end()].endPose.p;}

    Task & getTask(){ //returns Task being executed
        return currentTask;
    }

    std::vector <BodyFeatures> & world_objects(){
        return worldBuilder.get_world_objects();
    }

    TransitionSystem & get_ts(){
        return transitionSystem;
    }

    Task & getGoal(){
        return controlGoal;
    }

    void set_data2fp(const CoordinateContainer &data){
        data2fp=data;
    }

    int data_size(){
        return data2fp.size();
    }

    void clear_plan(){
        plan.clear();
    }
};



int main(int argc, char** argv){
    
    //we imagine that we have executed a plan and then the robot is instructed to go back on its steps
    Disturbance target1;
    std::vector <Direction> solution={DEFAULT}, solution2=solution, solution3=solution, solution4=solution;
    float simStep=0.5;
    if (argc>2){
        if (atoi(argv[2])==1){
            target1= Disturbance(PURSUE, b2Vec2(1.0,0), 0);  
            solution={DEFAULT, DEFAULT, LEFT, DEFAULT, RIGHT, DEFAULT, RIGHT, DEFAULT, LEFT, DEFAULT };  
            simStep=std::max(ROBOT_HALFWIDTH*2, ROBOT_HALFLENGTH*2);
            solution2={DEFAULT, LEFT, DEFAULT, RIGHT, DEFAULT, RIGHT, DEFAULT, LEFT, DEFAULT};  
            solution3={LEFT, DEFAULT, RIGHT,  DEFAULT, RIGHT, DEFAULT};  
            solution4={LEFT, DEFAULT, RIGHT,  DEFAULT, DEFAULT, RIGHT, DEFAULT};  
        }
        else{
            solution={LEFT, DEFAULT};
            solution2={RIGHT, DEFAULT};
            solution3=solution;
        }
    }
    Task goal(target1,DEFAULT);
    HorizonStarPlanner planner;
    DebugConfigurator conf;
    conf.register_planner(&planner);
    conf.init(goal);
    ClosedLoop_Tracker tracker;
    conf.register_tracker(&tracker);
    //conf.setSimulationStep();
    LIDAR_In ci;
    conf.registerInterface(&ci, NULL);
    DataInterface di(&ci);
    if (argc>1){
        di.set_folder(argv[1]);
        di.newScanAvail();          
    }
    conf.set_data2fp(ci.data2fp);
    conf.Spawner();
    // conf.data2fp = ci.data2fp;
    // conf.addIteration();
    // b2World world(b2Vec2(0,0));
    // boost::clear_vertex(conf.movingVertex, conf.transitionSystem);
    // conf.worldBuilder.world_objects=conf.worldBuilder.getFeatures(conf.data2fp, b2Transform_zero);
    // conf.dummy_vertex(conf.currentVertex);
    // conf.explorer(conf.currentVertex, conf.transitionSystem, world);
    // conf.ts_cleanup(conf.transitionSystem, conf.plan);
    // std::vector <vertexDescriptor> plan=conf.planner(conf.transitionSystem, conf.currentVertex);
    //std::vector <Direction> plan_d=getPlan(conf.transitionSystem, plan, conf.currentVertex);
    conf.printPlan();

    //if (conf.get_plan()!=solution && plan_d !=solution2 && plan_d!=solution3 && plan_d!=solution4){
      //  return 1;
    //}
    return 0;
}