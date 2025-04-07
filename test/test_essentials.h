#ifndef TEST_ESSENTIALS_H
#define TEST_ESSENTIALS_H
#include "configurator.h"
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <iomanip> 
#include <sstream> //for writing string into file, for checking, std::ostringstream
#include <ncurses.h>
#include <ctime>
#include <dirent.h>
#include <filesystem>
#define _USE_MATH_DEFINES

std::mutex ctr_mutex;

bool debug_draw(b2Vec2 * sensor_v, std::vector <b2Vec2> d ){
    char name_v[256], name_s[256], name_d[256];
    sprintf(name_s, "/tmp/debug_sensor_cli.txt");
    sprintf(name_d, "/tmp/debug_disturbance_cl.txt");
    FILE * f_s=fopen(name_s, "w");
    FILE * f_d=fopen(name_d, "w");
    for (int i=0; i<4; i++){
        fprintf(f_s, "%f\t%f\n",  sensor_v[i].x, sensor_v[i].y);            
    }
    for (b2Vec2 d_v: d){
        fprintf(f_d, "%f\t%f\n",  d_v.x, d_v.y);            
    }
                    
    fclose(f_s);
    fclose(f_d);
}
 
template <typename T>
bool debug_draw(std::vector <T> d , char* tag){
    char name_d[256];
    sprintf(name_d, "/tmp/debug_dist_%s.txt", tag);
    FILE * f_d=fopen(name_d, "w");
    for (T d_v: d){
        fprintf(f_d, "%f\t%f\n",  d_v.x, d_v.y);            
    }
    fclose(f_d);
}

bool debug_draw(b2World & w, int file){
    char name_v[256], name_s[256], name_d[256];
    sprintf(name_v, "/tmp/debug_chassis_%i.txt",file);
    sprintf(name_s, "/tmp/debug_sensor_%i.txt",file);
    sprintf(name_d, "/tmp/debug_disturbance_%i.txt",file);
    FILE * f_v=fopen(name_v, "w");
    FILE * f_s=fopen(name_s, "w");
    FILE * f_d=fopen(name_d, "w");
    for (auto b=w.GetBodyList(); b; b=b->GetNext()){
        if (b->GetUserData().pointer){
            for (b2Fixture* fixture=b->GetFixtureList();fixture; fixture=fixture->GetNext()){
                b2PolygonShape * poly=(b2PolygonShape*)fixture->GetShape();
                int ct= poly->m_count;
                for (int i=0; i<ct; i++){
                    b2Vec2* v=poly->m_vertices+i;
                    b2Vec2 world_point= b->GetWorldPoint(*v);
                    if (b->GetUserData().pointer==ROBOT_FLAG){
                        if (fixture->IsSensor()){
                            fprintf(f_s, "%f\t%f\n",  world_point.x, world_point.y);            
                        }
                        else{
                            fprintf(f_v, "%f\t%f\n",  world_point.x, world_point.y);            
                        }
                    }
                    else if (b->GetUserData().pointer==DISTURBANCE_FLAG){
                        fprintf(f_d, "%f\t%f\n",  world_point.x, world_point.y);            
                    }
                }
            }
        }
    }
    fclose(f_v);
    fclose(f_s);
    fclose(f_d);
}

std::vector <BodyFeatures> WorldBuilder::processData(const CoordinateContainer& points, const b2Transform& start){
    std::vector <BodyFeatures> result;
    std::vector <Pointf> ptset= set2vec(points);
    std::pair<bool,BodyFeatures> feature= bounding_box(ptset);
    if (feature.first){
        feature.second.pose.q.Set(start.q.GetAngle());
        result.push_back(feature.second);
    }
    return result;
}

void round_mat(b2Transform & t){
    t.p.x=round(t.p.x*100)/100;
    t.p.y=round(t.p.y*100)/100;
    t.q.Set(DEG_TO_RAD_K*round(t.q.GetAngle()*(1/DEG_TO_RAD_K)));
}
void print_matrix(const cv::Mat & m){
	std::cout << "M = " << std::endl << " "  << m << std::endl << std::endl;
}

void Configurator::explore_plan(b2World&world){
    pre_explore(transitionSystem, plan, currentTask.change);
    vertexDescriptor src=get_explore_start(transitionSystem);
    resetPhi(transitionSystem);
    std::vector <vertexDescriptor> plan_tmp=explorer(src, transitionSystem, world);
    if (DEBUG){
        std::vector<vertexDescriptor> _plan=(plan);
        debug::graph_file(iteration, transitionSystem, controlGoal.disturbance, _plan, currentVertex);
    }		
    ts_cleanup(transitionSystem, plan); //remove self-edge and singleton states
    if (plan_tmp.empty() && (!transitionSystem[currentVertex].visited() || currentTask.change)){ //currentv not visited means that it wasn't observed ()
        printf("no plan, searchign from %i\n", src);
        bool finished=false;
        plan_tmp= planner(transitionSystem, currentVertex, TransitionSystem::null_vertex(), false, NULL, &finished); //src
    }
    else{
        printf("recycled plan in explorer:\n");
    }
    plan=plan_tmp;
    printPlan(&plan);
}

//debug, to visualise 
void flush_points(const std::vector<std::vector<cv::Point2f>> clusters, char * where){
    char destination[256];
    int i=1;
    for (std::vector<cv::Point2f> v:clusters){
        sprintf(destination, "/tmp/%s_%04f.txt", where, i);
        FILE * f=fopen(destination, "w");
        for (cv::Point2f p:v){
            fprintf(f, "%f\t%f\n", p.x, p.y);
        }
        fclose(f);
        i++;
    }
}

struct Goal_Changer:public GoalChanger{
    void change_goal(Task * goal){
        EndedResult er = goal->checkEnded(b2Transform_zero, UNDEFINED, true);//true
        if (er.ended){ //& c->getTask()->motorStep<1
            Disturbance new_goal(PURSUE, b2Vec2(-1, 0), -1);
		    *goal = Task(new_goal, UNDEFINED);
	    }

    }
};



#endif