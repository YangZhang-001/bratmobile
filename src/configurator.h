#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
#include <algorithm>
#include <sys/stat.h>
#include "debug.h"
#include "planner.h"
#include "control_interface.h"
#include "task_controller.h"
#include "tracker.h"

class Configurator{
protected:
	int iteration=0; //represents that hasn't started yet, robot isn't moving and there are no map data
	Task currentTask; //need to make thread safe?
	Controller * task_controller=NULL;
	Tracker * tracker=NULL;
	LIDAR_In * ci=NULL;
	Motor_Out * control=NULL;
	Planner *planner=NULL;
	Logger * logger=NULL;
	bool running =0;
	std::thread * LIDAR_thread=NULL;
	float simulationStep=2*std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH);
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	GoalChanger * goal_changer=NULL;	
	std::vector<vertexDescriptor>plan, current_vertices;
	int bodies=0;
	Task controlGoal;
	CoordinateContainer data2fp;
	TransitionSystem transitionSystem=TransitionSystem(1);
	WorldBuilder worldBuilder;
	vertexDescriptor currentVertex=movingVertex;
	edgeDescriptor movingEdge=edgeDescriptor(), currentEdge=edgeDescriptor();

	public:


Configurator(){
	init();
}

Configurator(Task _task){
	init(_task);
}

/**
 * @brief Initialises configurator
 * 
 * @param _task the new overarching goal
 */
void init(Task _task=Task());

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


void dummy_vertex(vertexDescriptor src);

simResult simulate(Task, b2World &);

/**
*@param g the cognitive map
*@param t the task which is currently being executed on the robot
*/
void estimate_current_vertex();


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
std::pair<edgeDescriptor, bool> addVertex(const vertexDescriptor & src, vertexDescriptor &v1, TransitionSystem &g, Edge edge=Edge(), bool topDown=0){ //returns edge added
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

//search the TS for a plan
//std::vector <vertexDescriptor> planner(TransitionSystem&, vertexDescriptor, vertexDescriptor goal=TransitionSystem::null_vertex(), bool been=0, const Task* custom_ctrl_goal=NULL, bool * finished =NULL) ;

/**
 * @brief Explores and plan
 * 
 */
virtual void explore_plan(b2World&)=0;

//std::vector <vertexDescriptor> back_planner(TransitionSystem&, vertexDescriptor, vertexDescriptor root=0);

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

//round angle to a divisor of PI/2
float approximate_angle(const float &, const Direction &, const simResult::resultType &);


void adjust_goal_expectation();


void register_controller(Controller * controller){
	task_controller=controller;
}

Controller* get_controller(){
	return task_controller;
}

void register_tracker(Tracker * _tracker){
	if (!_tracker){return;}
	tracker=_tracker;
	tracker->init(&controlGoal);
}

Tracker * get_tracker()const {
	return tracker;
}

Motor_Out * get_motor_interface(){
	return control;
}

LIDAR_In * get_lidar_interface(){
	return ci;
}


void register_planner(Planner * _p){
	planner=_p;
}

void setSimulationStep(float f){
	simulationStep=f;
}

void register_logger(Logger * l){
	logger=l;
}

protected:
	// void log(char * format, ){
	// 	if (NULL!=logger){
	// 		va_list args;
	// 		//va_start(args, format);
	// 		logger->log(format, args);
	// 	}
	// }



};

/**
 * @brief Configurator with long-range planning. It explores transitions out of a state until a DEFAULT Task is reached.
 * The next state to expand will be the one with lowest heuristic cost. 
 * 
 */
class AttentiveConfigurator:public Configurator{
	protected:
	StateMatcher matcher;
/**
 * @brief Package task/goal execution details into an Execution Info instance
 * 
 * @param gv goal vertex
 * @param been has goal been visited
 * @return Planner::ExecutionInfo 
 */
ExecutionInfo package_info(vertexDescriptor gv=TransitionSystem::null_vertex(), bool been=false){
	return ExecutionInfo(currentVertex, gv, currentTask, controlGoal, been, plan);
}

/**
 * @brief Use attention window to find if any previously avoided obstacle is in the way of the goal, if present.
 * In case of plan recycling, it shifts the disturbance to adapt to the current task ahead
 * 
 * @param g the transition system
 * @param v source vertex
 * @param world box2d world
 * @param dir direction of the task to be simulated
 * @param start task start
 * @return previous Di if it's in the way of target, even though source task was successful
 * 		   previous Dn if previous task is safe for now
 * 		   goal Di otherwise 
 */
Disturbance getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start);


//add waypoints to proprity queue
void backtrack(std::vector <vertexDescriptor>&, std::vector <vertexDescriptor>&, const std::set<vertexDescriptor>&, TransitionSystem&, std::vector <vertexDescriptor>&);

//split this state into sub-state representing waypoints
std::vector <vertexDescriptor> splitTask(vertexDescriptor v, TransitionSystem&, Direction, vertexDescriptor src=TransitionSystem::null_vertex());

//if same task, it will terminate in the same disturbance
void propagateD(vertexDescriptor, vertexDescriptor, TransitionSystem&, std::vector<vertexDescriptor>*propagated=NULL, std::set<vertexDescriptor>*closed=NULL, StateMatcher::MATCH_TYPE match=StateMatcher::_FALSE);

//if in plan the vertex gets priority
void planPriority(TransitionSystem&, vertexDescriptor); 

void adjust_simulated_task(const vertexDescriptor&, TransitionSystem &, Task*);

//adjust real-world task
void adjust_rw_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);

//void recall_plan_from(const vertexDescriptor&, TransitionSystem & , b2World &, std::vector <vertexDescriptor>&, bool&, Disturbance *dist);

std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor>, TransitionSystem&);

std::pair <StateMatcher::MATCH_TYPE, vertexDescriptor> findMatch(State, TransitionSystem&, State * src, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, StateDifference * _sd=NULL); //matches to most likely

std::vector<vertexDescriptor> explorer(vertexDescriptor, TransitionSystem&, b2World &); //evaluates only after DEFAULT, internal one step lookahead

std::pair <bool, Direction> getOppositeDirection(Direction);

void resetPhi(TransitionSystem&g);

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
std::pair <edgeDescriptor, bool> add_vertex_now(const vertexDescriptor & src, vertexDescriptor & v1, TransitionSystem & g, Disturbance obs,Edge edge=Edge(), bool topDown=0);

/**
 * @brief Adds vertices retroactively (e.g. after a state is split)
 * 
 * @param src source state
 * @param v1 new state
 * @param g cognitive map
 * @param edge connecting edge between src->v1
 * @param topDown flag determining whether state v1 has been simulated already or not
 */
std::pair <edgeDescriptor, bool> add_vertex_retro(vertexDescriptor &src, vertexDescriptor &v1, TransitionSystem &g, Edge edge=Edge(), bool topDown=0);


//only keeps unexplored transitions out of vertex 

void unexplored_transitions(TransitionSystem&g, const vertexDescriptor& v);

/**
*Combines edges K and jump function: represents possible transitions out of a state
*@param v the vertex to which transitions are being assigned
*@param d state direction (redundant)
*@param src source vertex of state
*/
void transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src); 

/**
 * @brief Sets permitted transitions out of a state
 * 
 * @param g transitionSystem
 * @param v0 vertex descriptor for source state
 * @param d direction of state (to be removed later)
 * @param ended whether the overarching goal has ended
 * @param src source vertex of v0
 * @param plan_prov the plan
 */
void applyTransitionMatrix(TransitionSystem&g, vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov);

/**
 * @brief Adds vertexDescriptor  to priority queue according to a custom heuristic
 * 
 * @param v the vertex descriptor for the state
 * @param queue the priority queue
 * @param g the transition system
 * @param closed closed states: ones which have already been simulated and expanded
 */
void addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, TransitionSystem &g, const std::set <vertexDescriptor>& closed);


//removes singleton vertices and self-edges
void ts_cleanup(TransitionSystem &, std::vector <vertexDescriptor>&);

//apply affine transformation to states (e.g. if the same situation encountered in the past is reencountered)
void shift_states(TransitionSystem &, const std::vector<vertexDescriptor>&, const b2Transform &); //shifts a sequence of states by a certain transform

//return vertex from which exploration of the environment starts (explorer)
vertexDescriptor get_explore_start(TransitionSystem &);

/**
 * @brief Prepares transitionsystem for exploration: 
 * clears all edges of 	q0 with states that aren't the current one
 * and updates instantaneous state q0 (vertex 0) with info about the Task in execution 
 */
void pre_explore();

//reactive behaviour: simulate task to find disturbances and react to them
//void reactive(b2World&);

std::vector <State> output_plan(const std::vector<vertexDescriptor> &, const TransitionSystem &);

void explore_plan(b2World&)override;

/**
 * @brief Returns an iterator to the next option representing a discrete Task transition.
 * If v is in the plan, returns the next task in the plan. If this next task is successful, 
 * after that option has been simulated it returns iterator to vector end. If not, returns other options
 * 
 * @param v vertex being expanded
 * @param src v's source vertex
 * @param full_plan a plan
 * @return std::vector<Direction>::iterator 
 */
std::vector<Direction>::iterator  get_next_option(vertexDescriptor v, vertexDescriptor src, std::vector<vertexDescriptor> full_plan);

/**
 * @brief Assesses whether a previous plan can be recycled
 * 
 * @param v source vertex (start of module)
 * @param v0 vertex currently expanded
 * @param task_start vertex representing the start of the provisional plan
 * @param matchType match type of matching operation
 * @param shift_start vector to shift future start by
 * @param sk_first_start start of the state just simulated (sk.first)
 * @param edge edge between v0 and the new state (v1)
 * @param plan_prov provisional plan
 * @param t_get_direction direction of the task just simulated
 * @return true 
 * @return false 
 */
bool recycle_plan(vertexDescriptor &v, vertexDescriptor &v0, vertexDescriptor & task_start, StateMatcher::MATCH_TYPE &matchType, 
				b2Transform & shift_start, b2Transform& sk_first_start, std::pair<edgeDescriptor, bool>&edge,
				std::vector<vertexDescriptor> &plan_prov, Direction t_get_direction);


/**
 * @brief Sets up for simulation
 * 
 * @param W box2d world
 * @param t task (gets modified)
 * @param v0 source vertex for the next state
 * @param shift any shift to be applied (in case of plan recycling)
 * @param start task start
 * @param v0_options a subset of transitionSystem[0].options
 * @return sk,  pair of state and edge

 */
std::pair<State, Edge> simulation_setup(b2World& w, Task & t, vertexDescriptor v0, b2Transform shift, b2Transform &start, std::vector<Direction>v0_options);

void reassign_direction(vertexDescriptor bestNext, Direction& direction);

public:

AttentiveConfigurator(){};

AttentiveConfigurator(Task _task){
	init(_task);
}


};


/**
 * @brief Implements a Braitenberg vehicle: only simulates one Task at a time
 * 
 */
class ReactiveConfigurator:public Configurator{
	protected:
	void explore_plan(b2World&)override;
	public:

	ReactiveConfigurator(){};

	ReactiveConfigurator(Task _task){
		init(_task);
}
	

};

 #endif