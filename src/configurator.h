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
#include "task_controller.h"
#include "tracker.h"
//char bodyFile[100];

class Configurator{
protected:
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	Task currentTask; //need to make thread safe?
	Controller * task_controller=NULL;
	Tracker * tracker=NULL;
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

Configurator(Task _task){
	init(_task);
}

/**
 * @brief Initialises configurator
 * 
 * @param _task the new overarching goal
 */
void init(Task _task);

/**
 * @brief Calls functions to explore the state space and extract a plan
 * 
 * @return true 
 * @return false 
 */
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

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(State, TransitionSystem&, State * src, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, StateDifference * _sd=NULL); //matches to most likely

std::vector<vertexDescriptor> explorer(vertexDescriptor, TransitionSystem&, b2World &); //evaluates only after DEFAULT, internal one step lookahead

std::pair <bool, Direction> getOppositeDirection(Direction);

void resetPhi(TransitionSystem&g);

void printPlan(std::vector <vertexDescriptor>* p=NULL);

/**
 * @brief Add state to the cognitive map and set next state Task direction and options
 * 
 * @param src source state
 * @param v1 new state
 * @param g cognitive map
 * @param edge connecting edge between src->v1
 * @param topDown flag determining whether state v1 has been simulated already or not
 */
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
/**
 * @brief Adds state after discovering it in exploration
 * 
 * @param src source state
 * @param v1 new state
 * @param g cognitive map
 * @param Di the initial disturbance of v1
 * @param edge connecting edge between src->v1
 * @param topDown flag determining whether state v1 has been simulated already or not
 */
std::pair <edgeDescriptor, bool> add_vertex_now(vertexDescriptor & src, vertexDescriptor & v1, TransitionSystem & g, Disturbance obs,Edge edge=Edge(), bool topDown=0);

//adds vertex retroactively (e.g. if in split task)
std::pair <edgeDescriptor, bool> add_vertex_retro(vertexDescriptor &, vertexDescriptor &, TransitionSystem &, Disturbance,Edge edge=Edge(), bool topDown=0);

//search the TS for a plan
std::vector <vertexDescriptor> planner(TransitionSystem&, vertexDescriptor, vertexDescriptor goal=TransitionSystem::null_vertex(), bool been=0, const Task* custom_ctrl_goal=NULL, bool * finished =NULL) ;


//std::vector <vertexDescriptor> back_planner(TransitionSystem&, vertexDescriptor, vertexDescriptor root=0);

EndedResult estimateCost(State&, b2Transform, Direction); //returns whether the controlGoal has ended and fills node with cost and error

//calculates cumulative cost phi, add discount factor if in plan
float evaluationFunction(EndedResult, const vertexDescriptor &v, std::vector<vertexDescriptor>& p);

//starts thread
void start(); 

//stops thread
void stop(); 

void registerInterface(LIDAR_In *, Motor_Out *);

/**
 * @brief Spawns tasks, creates plans, tracks task and controls real-world task switching
 * option to run in thread. Thread can be used if planning time might exceed 200ms (LIDAR sampling rate)
 * but doesn't have to be
 * 
 */
static void run(Configurator *);

//only keeps unexplored transitions out of vertex v
void unexplored_transitions(TransitionSystem&, const vertexDescriptor& v);

/**
*Combines edges K and jump function: represents possible transitions out of a state
*@param state the state to which transitions are being assigned
*@param d state direction (redundant)
*@param src source vertex of state
*/
void transitionMatrix(State& state, Direction d, vertexDescriptor src); 

void applyTransitionMatrix(TransitionSystem&, vertexDescriptor, Direction,bool, vertexDescriptor, std::vector<vertexDescriptor>&);

void addToPriorityQueue(vertexDescriptor, std::vector <vertexDescriptor>&, TransitionSystem&, const std::set<vertexDescriptor>&);

void addToPriorityQueue(Frontier, std::vector <Frontier>&, TransitionSystem&, vertexDescriptor goal=TransitionSystem::null_vertex());


void setSimulationStep(float f){
	simulationStep=f;
}

//round angle to a divisor of PI/2
float approximate_angle(const float &, const Direction &, const simResult::resultType &);

//removes singleton vertices and self-edges
void ts_cleanup(TransitionSystem &, std::vector <vertexDescriptor>&);

//apply affine transformation to states (e.g. if the same situation encountered in the past is reencountered)
void shift_states(TransitionSystem &, const std::vector<vertexDescriptor>&, const b2Transform &); //shifts a sequence of states by a certain transform

//return vertex from which exploration of the environment starts (explorer)
vertexDescriptor get_explore_start(TransitionSystem &);

void pre_explore(TransitionSystem &, const std::vector<vertexDescriptor>&, const bool& );

//wrapper around environment explorer and planners
void explore_plan(b2World&);

//reactive behaviour: simulate task to find disturbances and react to them
//void reactive(b2World&);

std::vector <State> output_plan(const std::vector<vertexDescriptor> &, const TransitionSystem &);

//returns estimation of current vertex based on observed or estimate position of Di in currentTask
/**
*@param g the cognitive map
*@param t the task which is currently being executed on the robot
*/
void estimate_current_vertex(TransitionSystem&, Task& t);

// //uses LIDAR data to calculate an affine transform of disturbance Di if present
// void track_task_execution();

/**
 * @brief changes tasks to execute on
 */
void change_task();

//updates environment representation with time
/**
*@param g the cognitive map
*@param _deltaPose the transform to apply
*/
void update_graph(TransitionSystem& g, const b2Transform & _deltaPose);


//void react();

void adjust_goal_expectation();


void register_controller(Controller * controller){
	task_controller=controller;
}

void register_tracker(Tracker * _tracker){
	tracker=_tracker;
	tracker->init(&controlGoal);
}

Tracker * get_tracker()const {
	return tracker;
}

};



 #endif