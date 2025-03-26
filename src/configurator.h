#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
//#include "worldbuilder.h"
#include <algorithm>
#include <sys/stat.h>
#include "debug.h"
#include "planner.h"
#include "control_interface.h"
//char bodyFile[100];

class Configurator{
protected:
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	Task currentTask; //need to make thread safe?
public:
	LIDAR_In * ci=NULL;
	Motor_Out * control=NULL;
	bool running =0;
	std::thread * LIDAR_thread=NULL;
	float simulationStep=2*std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH);
	Task controlGoal;
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	CoordinateContainer data2fp;
	int bodies=0;
	TransitionSystem transitionSystem;
	StateMatcher matcher;
	WorldBuilder worldBuilder;
	vertexDescriptor movingVertex;
	vertexDescriptor currentVertex;
	edgeDescriptor movingEdge, currentEdge;
	std::vector<vertexDescriptor>plan, current_vertices;
	GoalChanger * goal_changer=NULL;	

Configurator()=default;

Configurator(Task _task): controlGoal(_task), currentTask(_task){
	previousTimeScan = std::chrono::high_resolution_clock::now();
	movingVertex=boost::add_vertex(transitionSystem);
	transitionSystem[movingVertex].Di=controlGoal.disturbance;
	currentVertex=movingVertex;
	currentTask.action.setVelocities(0,0);
	gt::fill(simResult(), &transitionSystem[movingVertex]);
}


bool Spawner(); 

int getIteration(){
	return iteration;
}

void addIteration(int i=1){
	iteration+=i;
}

Task * getTask(int advance=0){ //returns Task being executed
	return &currentTask;
}

void dummy_vertex(vertexDescriptor src);


//inputs: g, src vertex, b2d world, direction of the task to be created
Disturbance getDisturbance(TransitionSystem&, const vertexDescriptor&, b2World &, const Direction &, const b2Transform&);


simResult simulate(Task, b2World &);

//add waypoints to proprity queue
void backtrack(std::vector <vertexDescriptor>&, std::vector <vertexDescriptor>&, const std::set<vertexDescriptor>&, TransitionSystem&, std::vector <vertexDescriptor>&);

//split this state into sub-state representing waypoints
std::vector <vertexDescriptor> splitTask(vertexDescriptor v, TransitionSystem&, Direction, vertexDescriptor src=TransitionSystem::null_vertex());

//if same task, it will terminate in the same disturbance
void propagateD(vertexDescriptor, vertexDescriptor, TransitionSystem&, std::vector<vertexDescriptor>*propagated=NULL, std::set<vertexDescriptor>*closed=NULL, StateMatcher::MATCH_TYPE match=StateMatcher::_FALSE);

//void pruneEdges(std::vector<std::pair<vertexDescriptor, vertexDescriptor>>, TransitionSystem&, vertexDescriptor&, vertexDescriptor&,std::vector <vertexDescriptor>&, std::vector<std::pair<vertexDescriptor, vertexDescriptor>>&); //clears edges out of redundant vertices, removes the vertices from PQ, returns vertices to remove at the end

//if in plan the vertex gets priority
void planPriority(TransitionSystem&, vertexDescriptor); 

void adjust_simulated_task(const vertexDescriptor&, TransitionSystem &, Task*);

//adjust real-world task
void adjust_rw_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);

//finds frontier: closest states with DEFAULT tasks reachable from a vertex v
std::vector <Frontier> frontierVertices(vertexDescriptor, TransitionSystem&, Direction , bool been=0); //returns the closest vertices to the start vertex which are reached by executing a task of the specified direction

//void recall_plan_from(const vertexDescriptor&, TransitionSystem & , b2World &, std::vector <vertexDescriptor>&, bool&, Disturbance *dist);

std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor>, TransitionSystem&);

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(State, TransitionSystem&, State * src, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, std::vector <vertexDescriptor>* others=NULL, bool relax=0, bool wholeTask=false); //matches to most likely

std::vector<vertexDescriptor> explorer(vertexDescriptor, TransitionSystem&, b2World &); //evaluates only after DEFAULT, internal one step lookahead

std::pair <bool, Direction> getOppositeDirection(Direction);

void resetPhi(TransitionSystem&g);

void printPlan(std::vector <vertexDescriptor>* p=NULL);

std::pair<edgeDescriptor, bool> addVertex(vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Edge edge=Edge(), bool topDown=0){ //returns edge added
	std::pair<edgeDescriptor, bool> result;
	result.second=false;
	if (g[src].options.size()>0 || topDown){
		v1 = boost::add_vertex(g);
		result = add_edge(src, v1, g);
		g[result.first] =edge;
		g[v1].direction=g[src].options[0];
		g[result.first].it_observed=iteration;
		if (!topDown){
			g[src].options.erase(g[src].options.begin());
		}

	}
	return result;
}

//adds vertex after discovering it in exploration
std::pair <edgeDescriptor, bool> add_vertex_now(vertexDescriptor &, vertexDescriptor &, TransitionSystem &, Disturbance,Edge edge=Edge(), bool topDown=0);

//adds vertex retroactively (e.g. if in split task)
std::pair <edgeDescriptor, bool> add_vertex_retro(vertexDescriptor &, vertexDescriptor &, TransitionSystem &, Disturbance,Edge edge=Edge(), bool topDown=0);

//search the TS for a plan
std::vector <vertexDescriptor> planner(TransitionSystem&, vertexDescriptor, vertexDescriptor goal=TransitionSystem::null_vertex(), bool been=0, const Task* custom_ctrl_goal=NULL, bool * finished =NULL) ;


//std::vector <vertexDescriptor> back_planner(TransitionSystem&, vertexDescriptor, vertexDescriptor root=0);

EndedResult estimateCost(State&, b2Transform, Direction); //returns whether the controlGoal has ended and fills node with cost and error

float evaluationFunction(EndedResult);

void start(); //data interface class collecting position of bodies

void stop();

void registerInterface(LIDAR_In *, Motor_Out *);

static void run(Configurator *);

// static void set_motor_output(Configurator *); //sending tracking info to motor IO interface

// static void get_motor_input(Configurator *); //sending tracking info to motor IO interface

void unexplored_transitions(TransitionSystem&, const vertexDescriptor&);

void transitionMatrix(State&, Direction, vertexDescriptor); //DEFAULT, LEFT, RIGHT

void applyTransitionMatrix(TransitionSystem&, vertexDescriptor, Direction,bool, vertexDescriptor, std::vector<vertexDescriptor>&);

void addToPriorityQueue(vertexDescriptor, std::vector <vertexDescriptor>&, TransitionSystem&, const std::set<vertexDescriptor>&);

void addToPriorityQueue(Frontier, std::vector <Frontier>&, TransitionSystem&, vertexDescriptor goal=TransitionSystem::null_vertex());


void setSimulationStep(float f){
	simulationStep=f;
}

float approximate_angle(const float &, const Direction &, const simResult::resultType &);

void ts_cleanup(TransitionSystem &, std::vector <vertexDescriptor>&);

void shift_states(TransitionSystem &, const std::vector<vertexDescriptor>&, const b2Transform &); //shifts a sequence of states by a certain transform

vertexDescriptor get_explore_start(TransitionSystem &);

void pre_explore(TransitionSystem &, const std::vector<vertexDescriptor>&, const bool& );

void explore_plan(b2World&);

void reactive(b2World&);

std::vector <State> output_plan(const std::vector<vertexDescriptor> &, const TransitionSystem &);

vertexDescriptor estimate_current_vertex(TransitionSystem&, Task& currentTask, vertexDescriptor currentVertex);

void track_task_execution(); //returns observed disturbance

void change_task();

int motor_step(Task::Action a);

void update_graph(TransitionSystem&, const b2Transform & _deltaPose, Task* t, Task * goal);

//merge vertices into a single task
Task task_to_execute(const std::vector<vertexDescriptor>&, const TransitionSystem&, int);

//void makeRobotSensor(TransitionSystem&, const vertexDescriptor&, const Task& t); //sensor but not linked to a body

int to_task_end();
private:
b2PolygonShape task_sensor;

};



 #endif