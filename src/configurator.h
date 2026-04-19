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
#include "config_callbacks.h"
#include "task_controller.h"
#include "tracker.h"

class Configurator{
protected:
	int iteration=0, simulatedTasks=0; 
	Task currentTask;
	Controller * task_controller=NULL;
	Tracker * tracker=NULL;
	MotorInterface * control=NULL;
	Planner *planner=NULL;
	Logger * logger=NULL;
	bool running =0;
	std::thread * LIDAR_thread=NULL;
	float simulationStep=2*std::max(ROBOT_HALFLENGTH, ROBOT_HALFWIDTH);
	std::chrono::high_resolution_clock::time_point previousTimeScan;
	GoalChanger * goal_changer=NULL;
	CalibrationCallback * calibrationCallback=NULL;	
	std::vector<vertexDescriptor>m_plan, current_vertices;
	Task controlGoal;
	CoordinateContainer data2fp;
	TransitionSystem transitionSystem=TransitionSystem(1);
	WorldBuilder *worldBuilder=new WorldBuilder();
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
	delete worldBuilder;
	worldBuilder=NULL;
}

/**
 * @brief Inserts one xy coordinate in the coordinate container
 */
void insertCoordinate(float x, float y){
	data2fp.insert(Pointf(x, y));
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

void assignDisturbanceToTask(const Disturbance & d, Task & t);


void dummy_vertex(vertexDescriptor src);

/**
 * @brief Simulates this task, includes creating the robot object
 * 
 * @param t the task (should already be initialised)
 * @param w the world (should already contain any objects aside from the robot)
 * @return simResult 
 */
simResult simulate(Task t, b2World & w);

virtual float remainingSimulationTime(const Task * const t=NULL);

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

// //starts thread
// void start(); 

// //stops thread
// void stop(); 

void registerInterface(MotorInterface *);

/**
 * @brief Spawns tasks, creates plans, tracks task and controls real-world task switching
 * option to run in thread. Thread can be used if planning time might exceed 200ms (LIDAR sampling rate)
 * but doesn't have to be
 * 
 */
void newScanEvent();

/**
 * @brief Clears coordinates
 */
void clearData(){data2fp.clear();}


/**
 * @brief changes tasks executing on the robot
 */
virtual void change_task();

/**
 * @brieF updates the cognitive map and goal by applying a 2D transform, and sets current task Di to the observed disturbance in the tracking result
*@param g the cognitive map
*@param tr the tracking result
*/
void update_graph(TransitionSystem& g, const TrackingResult & tr);

//round angle to a divisor of PI/2
/**
 * @brief Approximating angle to divisors of pi/2
 * @param angle the angle to approximate
 * @param d the direction of the robot
 * @param outcome the outcome of the last simulation
 * 
 */
float approximate_angle(float angle, Direction d, simResult::resultType outcome);

/**
* @brief Uses tracking information to adjust the position of the goal relative to the robot
*/
virtual void adjust_goal_expectation();

/**
 * @return std::pair <bool, Direction>(opposite exists, opposite direction)
 */
std::pair <bool, Direction> getOppositeDirection(Direction d);


/**
 * @brief If Task @param t corresponds to current Task, change its end criteria so that it is only simulated for the remainder of the end criteria
 * 
 * @param v source vertex for the task
 * @param t task reference
 */
void adjust_simulated_task(const vertexDescriptor&v,  Task& t);


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
	tracker->init(controlGoal);
}

Tracker * get_tracker()const {
	return tracker;
}

MotorInterface * get_motor_interface(){
	return control;
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

void register_goalChanger(GoalChanger * gc){
	goal_changer=gc;
}

void registerCalibrationCallback(CalibrationCallback * cb){
	calibrationCallback=cb;
}

/**
 * @brief Matrix multiply by transpose
 * 
 */
static void MulT(const b2Transform& B, Task & task);


/**
 * @brief Matrix multiply by transpose
 * 
 */
static void InvMul(const b2Transform& B, Task & task);

/**
 * @brief Matrix multiply by transpose
 * 
 */
static void InvMul_(const b2Transform& B, Task & task);

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
virtual Robot makeRobot(b2World& world, const Task & task);

//Disturbance * getGoalDisturbance(){return &controlGoal.disturbance;}

bool areInterfacesSetUp();

/**
 * @brief Assigns body features to the disturbance of a task (NOTE: affordance and validity of the disturbance will remain the same)
 * 
 * @param t the task
 * @param bf body features
 */
void assignBodyFeatures(Task & t, const BodyFeatures & bf);

/**
 * @brief Assigns dimensions to the disturbance of a task (NOTE: pose, affordance and validity of the disturbance will remain the same)
 * 
 * @param t the task
 * @param halfLength 
 * @param halfWidth 
 */
void assignDimensions(Task & t, float halfLength, float halfWidth);

void register_worldBuilder(WorldBuilder * wb){
	if (worldBuilder){
		delete worldBuilder;
	}
	worldBuilder=wb;
}

b2Transform get_start(const Task *const t){
	if (!t){
		throw std::invalid_argument("null pointer to task");
	}
	return t->start;	
}

Direction get_direction(const Task *const t){
	if (!t){
		throw std::invalid_argument("null pointer to task");
	}
	return t->direction;	
}

/**
 * @brief Does this Task correspond to the Task being executed?
 * Checks 1) if directions match, 2) if initial Disturbance Di match in their affordance, 
 * 3) if the Task starts at the origin of the Cartesian plane (current robot position)
 */
bool isCurrentTask(const Task & t);
};

/**
 * @brief Implements a Braitenberg vehicle: only simulates one Task at a time
 * 
 */
class ReactiveConfigurator:public Configurator{
	protected:
	void explore_plan(b2World&)override;

	float remainingSimulationTime(const Task *const t)override;

	public:

	ReactiveConfigurator(){};

	ReactiveConfigurator(Task _task){
		init(_task);
}
	

};

 #endif