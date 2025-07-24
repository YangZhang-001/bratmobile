#ifndef CONFIGURATOR_H
#define CONFIGURATOR_H
#include <dirent.h>
#include <thread>
#include <filesystem>
#include <ncurses.h>
#include <fstream>
#include <algorithm>
#include <random>
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
	std::vector<vertexDescriptor>m_plan, current_vertices;
	int bodies=0;
	Task controlGoal;
	CoordinateContainer data2fp;
	TransitionSystem transitionSystem=TransitionSystem(1);
	WorldBuilder worldBuilder;
	vertexDescriptor currentVertex=MOVING_VERTEX;
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
 * @param edge connecting edge between src->v1
 * @param topDown flag determining whether state v1 has been simulated already or not
 */
std::pair<edgeDescriptor, bool> addVertex(const vertexDescriptor & src, vertexDescriptor &v1, Edge edge=Edge(), bool topDown=0);

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

/**
 * @brief Matrix multiply by transpose
 * 
 */
static void MulT(const b2Transform& B, Task & task);

/**
 * @brief Matrix multiplication
 * 
 */
static void Mul(const b2Transform& B, Task &task);


protected:


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
	return ExecutionInfo(currentVertex, gv, currentTask, controlGoal, been, m_plan);
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

/**
 * @brief Iterates through vertices, if they result in crash, it splits the tasks and recalculates
 * evaluation functions and adds to priority queue
 * 
 * @param evaluation_q evaluation queue: lists of vertices to be evaluated for splitting
 * @param priority_q the priority queue to add vertices to
 * @param closed closed set 
 * @param plan_prov provisional plan
 * @param module_src source vertex from which modular expansion began
 * @param startRecycle vertex from which plan recycling started (for correcting pq)
 */
void backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src=MOVING_VERTEX, vertexDescriptor startRecycle=MOVING_VERTEX);

/**
 * @brief Split tasks into sub-states of fixed length
 * 
 * @param v the vertex to split
 * @param d direction of the task to split
 * @param src source vertex for v
 * @return std::vector <vertexDescriptor> : the vertices making up substates in the original task
 */
std::vector <vertexDescriptor> splitTask(vertexDescriptor v, Direction d, vertexDescriptor src=TransitionSystem::null_vertex());

/**
 * @brief Propagate a disturbance backwards to all states representing the same task
 * 
 * @param v1 final vertex linked to the final sub-state in the task
 * @param v0 source of v1
 * @param closed closed set
 * @param match is v1 a match of any kind to a vertex in the graph
 */
void propagateD(vertexDescriptor v1, vertexDescriptor v0, std::set<vertexDescriptor>*closed=NULL, StateMatcher::MATCH_TYPE match=StateMatcher::_FALSE);

//if in plan the vertex gets priority
void planPriority(TransitionSystem&, vertexDescriptor); 

/**
 * @brief If Task @param t corresponds to current Task, change its end criteria so that it is only simulated for the remainder of the end criteria
 * 
 * @param v source vertex for the task
 * @param t task reference
 */
void adjust_simulated_task(const vertexDescriptor&v,  Task& t);

//adjust real-world task
void adjust_rw_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);

/**
 * @brief Return edge with maximum probablitit in a vector
 * 
 * @param ev vector of edges
 * @return std::pair <edgeDescriptor, bool> 
 */
std::pair <edgeDescriptor, bool> maxProbability(std::vector<edgeDescriptor> ev, TransitionSystem&);

/**
 * @brief Searches transition System for a match to a state provided. Continuous states are
 * matched, with the option to also match the discrete state (the direction)
 * 
 * @param s the state to find a match for
 * @param dir simulated task direction (allowing to match continuous states only)
 * @param match_type fuzzy operator indicating what parameters in the continuous state match
 * @param _sd pointer to state difference, can be used for further calculatins
 * @param other_matches pointer to a vector of other matches (all of the type defined by @param match_type)
 * @return VertexMatch the best match found and its type
 */
VertexMatch findMatch(State s, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, StateDifference * _sd=NULL, std::vector <VertexMatch>*other_matches=NULL); //matches to most likely

/**
 * @brief Constructs transition system using a Box2D simulation combined with an A* graph
 * expansion algorithm
 * 
 * @param v starting vertex
 * @param g the transition system
 * @param w box2d world
 * @return std::vector<vertexDescriptor> a plan, if recycled from previous knowledge
 */
std::vector<vertexDescriptor> explorer(vertexDescriptor v, TransitionSystem&g, b2World &w); //evaluates only after DEFAULT, internal one step lookahead

/**
 * @return std::pair <bool, Direction>(opposite exists, opposite direction)
 */
std::pair <bool, Direction> getOppositeDirection(Direction);

/**
 * @brief Resets all vertices evaluation function phi to a default unitialised value of 10
 */
void resetPhi();

/**
 * @brief Adds state after discovering it in exploration
 * 
 * @param src source state
 * @param v1 new state
 * @param Di the initial disturbance of v1
 * @param edge connecting edge between src->v1
 * @param topDown flag determining whether state v1 has been simulated already or not
 */
std::pair <edgeDescriptor, bool> add_vertex_now(const vertexDescriptor & src, vertexDescriptor & v1, Disturbance obs,Edge edge=Edge(), bool topDown=0);

/**
 * @brief Adds vertices retroactively (e.g. after a state is split)
 * 
 * @param src source state
 * @param v1 new state
 * @param edge connecting edge between src->v1
 * @param topDown flag determining whether state v1 has been simulated already or not
 */
std::pair <edgeDescriptor, bool> add_vertex_retro(vertexDescriptor &src, vertexDescriptor &v1, Edge edge=Edge(), bool topDown=0);

/** * returns a vector of which directions in vector @param directions were explored (at the present iteration) from vertex @param v */
std::vector <Direction> getExploredDirections(vertexDescriptor v, const std::vector<Direction>& directions);

/** * only keeps unexplored transitions out of vertex @param v*/
void removeExploredTransitions(vertexDescriptor v);

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
 * @param v0 vertex descriptor for source state
 * @param d direction of state (to be removed later)
 * @param ended whether the overarching goal has ended
 * @param src source vertex of v0
 * @param plan_prov the plan
 */
void applyTransitionMatrix(vertexDescriptor v0, Direction d, bool ended, vertexDescriptor src, std::vector<vertexDescriptor>& plan_prov);

/**
 * @brief Adds vertexDescriptor  to priority queue according to a custom heuristic
 * 
 * @param v the vertex descriptor for the state
 * @param queue the priority queue
 * @param g the transition system
 * @param closed closed states: ones which have already been simulated and expanded
 */
void addToPriorityQueue(vertexDescriptor v, std::vector<vertexDescriptor>& queue, const std::set <vertexDescriptor>& closed);


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


//std::vector <State> output_plan(const std::vector<vertexDescriptor> &, const TransitionSystem &);

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
bool recycle_plan(vertexDescriptor v, vertexDescriptor &v0, vertexDescriptor & task_start, StateMatcher::MATCH_TYPE &matchType, 
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

/**
 * @brief Reassigns direction as the direction of the bext task to expand next in explorer
 * 
 * @param bestNext vertices representing task with lowest phi
 * @param direction direction to reassign
 */
void reassign_direction(vertexDescriptor bestNext, Direction& direction);

/**
 * @brief  if the match is a crashed task
 * 
 * @param match 
 * @param other_matches 
 * @return true if changes match
 */
bool matchToSafe(VertexMatch &match,const std::vector<VertexMatch> &other_matches=std::vector<VertexMatch>());

/**
 * @brief Given a valid match, sets up the edge with the previous vertex
 * Creates new edge if it doesn't exist, changes the match to a safe state
 * and allow the edge to be used in planning
 * 
 * @param match 
 * @param v0 
 * @param v1 
 * @param k 
 * @param direction 
 * @return std::pair<edgeDescriptor, bool> 
 */
std::pair<edgeDescriptor, bool> setup_match_edge(VertexMatch &match, vertexDescriptor &v0, vertexDescriptor & v1,const Edge& k, Direction direction, bool changedMatch);

/**
 * @brief Rerturns all the vertices making up a task
 * 
 * @param v a vertex representing a state
 * @param ep connecting edge to the task
 */
std::vector <vertexDescriptor> task_vertices(vertexDescriptor v, std::pair<bool, edgeDescriptor>* ep=NULL);

/**
 * @brief Returns a visited edge if present, or if the current 
 * 
 * @param es 
 * @param g 
 * @param cv 
 * @return std::vector <vertexDescriptor> 
 */
std::vector <vertexDescriptor> visitedOrVisitingEdge(const std::vector <edgeDescriptor>& es, TransitionSystem& g, vertexDescriptor cv=TransitionSystem::null_vertex());

/**
 * @brief Returns the vertex from which to start recycling plan
 * 
 * @param v source vertex which is being expanded
 * @param v1 last vertex in task, or the match
 * @param taskStart start of the task
 * @return @param v if the task is successful, @param connectingEdge if it ends in crash
 */
vertexDescriptor getRecyclingStart(vertexDescriptor v, vertexDescriptor v1, vertexDescriptor taskStart);

/**
 * @brief Returns a vector of all the in-edges of vertex @param v. Option to enter 
 * @param d to select a subset of edge. Does not return self-edges
 * @return std::vector <edgeDescriptor>
 */
std::vector <edgeDescriptor> inEdges(vertexDescriptor v, Direction d = UNDEFINED); //returns a vector containing all the in-edges of a vertex which have the specified direction

bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v);

/**
 * @brief Adds edge retrospectively (used in split task)
 * 
 * @param v source 
 * @param v1 target
 * @param s_tmp_endPose endPose for the new sub-state
 * @param first_edge original unsplit task edge
 * @param Direction d
 * @return std::pair<edgeDescriptor, bool> 
 */
std::pair<edgeDescriptor, bool> addEdgeRetrospectively(vertexDescriptor v, vertexDescriptor &v1, const State & s_tmp,std::pair<edgeDescriptor, bool> first_edge, Direction d, float linearSpeed);

/**
 * @brief Edits @param v out of the queue if the plan was recycled from a different vertex.
 * Useful if the frontier of @param v results in a crash and a plan needs to be recycled from the state
 * previous to it
 * 
 * @param queue queue
 * @param v source v (which may have a crash in its frontier)
 * @param startRecycle another state found to precede the frontier
 * @param planProvSize size of the provisional plan: indicates if the recycling was successful or not
 */
void correctQueue(std::vector<vertexDescriptor>& queue, vertexDescriptor v, vertexDescriptor startRecycle, int planProvSize);

/**
 * @brief Adjusts the probability that a continuous state (edge target) will occur after taking a discrete state transition from the edge source
 * 
 * @param e an edge descriptor
 */
void adjustProbability(const edgeDescriptor &e);

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