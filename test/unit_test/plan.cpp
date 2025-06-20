#include "test_classes.h"

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
    DebugConfigurator conf;
    conf.init(goal);
    ClosedLoop_Tracker tracker;
    conf.register_tracker(&tracker);
    conf.setSimulationStep(simStep);
    LIDAR_In ci;
    conf.registerInterface(&ci, NULL);
    DataInterface di(&ci);
    if (argc>1){
        di.set_folder(argv[1]);
        di.newScanAvail();          
    }
    conf.set_data2fp(ci.data2fp);
    conf.Spawner();
    std::vector <Direction> plan_d=getPlan(conf.get_ts(), conf.get_plan(), conf.get_current_vertex());
    if (plan_d!=solution && plan_d !=solution2 && plan_d!=solution3 && plan_d!=solution4){
        return 1;
    }
    return 0;
}