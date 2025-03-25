#ifndef CALLBACKS_H
#define CALLBACKS_H
#include "CppTimer.h"
#include <thread>
#include "test_essentials.h"

void printGraph(TransitionSystem& g){ //for calling in GDB
    boost::print_graph(g);
}

struct Remember{
	Remember(){}
	Remember(TransitionSystem* ts):g(ts){}

	bool operator()(const edgeDescriptor& e){//const
		if ((*g)[e].probability<FORGET_THRESHOLD){ //filter signal
		 	return false;
		 }
		return true;
	}

	private: 
	TransitionSystem *g;
}remember;

Visited visited;

template <typename Predicate> 
void printEdges(TransitionSystem& g, Predicate p){
    auto es = boost::edges(g);
    for (auto ei=es.first; ei!=es.second;ei++){
        if (!p(*ei)){
            printf("%i->%i, direction=%i,probability=%f, step=%i\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability, g[*ei].step);
        }
    }
}

void printEdges(TransitionSystem& g){
    auto es = boost::edges(g);
    for (auto ei=es.first; ei!=es.second;ei++){
        printf("%i->%i, direction=%i,probability=%f, step=%i\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability, g[*ei].step);
    }
}

void print_forget(TransitionSystem& g){
    Remember p;
    printEdges(g, p);
}

float print_belowP(TransitionSystem& g, float p){
    auto es = boost::edges(g);
    float ct=0;
    for (auto ei=es.first; ei!=es.second;ei++){
        if (g[*ei].probability<p){
            ct++;
            printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
        }
    }
    return ct/g.m_vertices.size();
}

void getVisited(TransitionSystem& g, vertexDescriptor cv){
    auto es = boost::edges(g);
    float ct=0;
    for (auto ei=es.first; ei!=es.second;ei++){
        if ((g[(*ei).m_source].visited()|| (*ei).m_source==0 || (*ei).m_source==cv)& g[(*ei).m_target].visited()){
            ct++;
            printf("%i->%i, direction=%i,probability=%f\n", (*ei).m_source, (*ei).m_target, g[(*ei).m_target].direction, g[*ei].probability);
        }
    }
}

class DataInterface {
public:
int iteration = 0;
LIDAR_In * ci;

char * folder;
    DataInterface(LIDAR_In * _ci): ci(_ci){}

	bool newScanAvail(){ //uncomment sections to write x and y to files		
        iteration++;
    	ci->setReady(false);
      //  ci->updatePCProc();
	//	ci->data.clear();
		ci->data2fp.clear();
		char filePath[256];
        char folderName[256];
        sprintf(folderName,"%s", folder);
        if (folderName != "empty"){
            sprintf(filePath, "%smap%04d.dat", folderName, iteration);
            printf("%s\n", filePath);
            FILE *f;
            if (!(f=fopen(filePath, "r"))){
                if (iteration>1){
                    iteration=1;
                }
                else{
                    ci->stop=1;
                    return false;
                }
            }
            else {
                fclose(f);
            }
            std::ifstream file(filePath);
            float x2, y2;
            while (file>>x2>>y2){
                x2 = round(x2*100)/100;
                y2 = round(y2*100)/100;
                Pointf  p2(x2,y2);
                ci->data2fp.insert(p2);
            }
            file.close();
        }
        ci->setReady(1);
        ci->iteration++;
        return true;
		


	}
};

class StepCallback{
    float L=0,R=0;
    Configurator * c;
    int ogStep=0;
public:

    StepCallback()=default;

    StepCallback(Configurator * _c): c(_c){}
    void step(){
        if (!c->running ){
            return;
        }
        c->control->track_task_execution(*c->getTask(), c->transitionSystem, &(c->controlGoal), c->currentVertex, c->data2fp);
        Task::Action action= c->getTask()->getAction();
        EndedResult er = c->controlGoal.checkEnded(b2Transform(b2Vec2(0,0), b2Rot(0)), UNDEFINED, true);//true
        if (er.ended &( c->getTask()->motorStep<1 & c->transitionSystem[c->currentVertex].direction!=STOP && c->control->plan.empty() && c->getIteration()>1)){ //& c->getTask()->motorStep<1
            Disturbance new_goal(PURSUE, c->controlGoal.start.p, c->controlGoal.start.q.GetAngle());
		    c->controlGoal = Task(new_goal, UNDEFINED);
            b2Vec2 v = c->controlGoal.disturbance.getPosition() - b2Vec2(0,0);
        	FILE * f = fopen(c->statFile, "a+");
            fprintf(f, "!");
            fclose(f);

	    }
	    c->control->change_task(c->getTask()->change,  c->control->plan,c->transitionSystem, c->controlGoal, *c->getTask(), c->currentVertex);
        L=c->getTask()->getAction().getLWheelSpeed();
        R= c->getTask()->getAction().getRWheelSpeed();
    }
};


//FOR THREAD DEBUGGING

class TimerStep: public CppTimer{
    StepCallback sc;
    public:
    TimerStep(StepCallback & _step): sc(_step){}
    void timerEvent(){
        sc.step();
    }
};

class TimerDI: public CppTimer{
    DataInterface di;
    public:
    TimerDI(DataInterface & _di): di(_di){}
    void timerEvent(){
        if (!di.newScanAvail()){
            this->stop();
        }
    }
};


 #endif