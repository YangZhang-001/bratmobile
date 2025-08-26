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
	int iteration=0, simulatedTasks=0; 
	Task currentTask;
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
	Task controlGoal;
	CoordinateContainer data2fp;
	TransitionSystem transitionSystem=TransitionSystem(1);
	WorldBuilder worldBuilder;
	vertexDescriptor currentVertex=MOVING_VERTEX;
	edgeDescriptor movingEdge=edgeDescriptor(), currentEdge=movingEdge;

	public:


Configurator(){
	init();
}

Configurator(Task _task){
	init(_task);
}

virtual ~Configurator(){
	stop();
}

/**
 * @brief Initialises configurator
 * 
 * @param _task the new overarching goal
 */
virtual void init(Task _task=Task());

/**
 * @brief Calls functions to explore the state space and extract a plan
 * 
 * @return true 
 * @return false 
 */
virtual bool Spawner(); 

int getIteration(){
	return iteration;
}

void addIteration(int i=1){
	iteration+=i;
}


void dummy_vertex(vertexDescriptor src);

/**
 * @brief Simulates this task, includes creating the robot object
 * 
 * @param t the task (should already be initialised)
 * @param w the world (should already contain any objects aside from the robot)
 * @return simResult 
 */
simResult simulate(Task t, b2World & w);

float remainingSimulationTime();

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

/**
 * @brief changes tasks executing on the robot
 */
void change_task();

/**
 * @brieF updates the cognitive map by applying a 2D transform
*@param g the cognitive map
*@param _deltaPose the transform to apply
*/
void update_graph(TransitionSystem& g, const b2Transform & _deltaPose);

//round angle to a divisor of PI/2
/**
 * @brief Approximating angle to divisors of pi/2
 * @param angle the angle to approximate
 * @param d the direction of the robot
 * @param outcome the outcome of the last simulation
 * 
 */
float approximate_angle(float angle, Direction d, simResult::resultType outcome);


void adjust_goal_expectation();


void register_controller(Controller * controller){
	task_controller=controller;
}

Controller* get_controller(){
	return task_controller;
}
/**
* @brief registers and initialises the tracker to the goal
*/
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

/**
 * @brief Makes the robot object in the world with the given start position and task
 * 
 * @param world 
 * @param start 
 * @param taskWithGoal 
 * @return Robot 
 */
virtual Robot makeRobot(b2World& world, const b2Transform & start);

Disturbance * getGoalDisturbance(){return &controlGoal.disturbance;}

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