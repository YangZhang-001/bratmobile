#ifndef PLANNER_H
#define PLANNER_H

#include "task.h"
/** \file */


    /**
     * @brief Information about what the configurator is doing (current Task, current vertex), what it wants to do (overarching goal), and whether it has done it before (been, goal vertex) 
     * 
     */
    class ExecutionInfo{
        private:
        vertexDescriptor m_currentVertex=0, m_goalVertex=TransitionSystem::null_vertex();
        Task m_currentTask;
        Task m_overarchingGoal; 
        bool m_been; //has a plan been made to fulfill this overarching goal before?
        std::vector <vertexDescriptor> m_plan;
        protected:
        friend class Configurator;
        friend class Explorer;

        void been(bool b){m_been=b;}

        void goalVertex(vertexDescriptor gv){m_goalVertex=gv;}
        
        void overarchingGoal(const Task & og){m_overarchingGoal=og;}

        public:
        ExecutionInfo(){}

        ExecutionInfo( vertexDescriptor _cv, vertexDescriptor _goal, Task & _ct, Task & _gt, bool _been, std::vector<vertexDescriptor> _plan){
            m_currentVertex=_cv;
            m_goalVertex=_goal;
            m_currentTask=_ct;
            m_overarchingGoal=_gt;
            m_been=_been;
            m_plan=_plan;
        }


        vertexDescriptor currentVertex()const{return m_currentVertex;}

        vertexDescriptor goalVertex()const{return m_goalVertex;}

        Task& currentTask(){return m_currentTask;}

        Task& overarchingGoal() {return m_overarchingGoal;}

        bool been()const{return m_been;}

        std::vector<vertexDescriptor> plan()const{return m_plan;}

    };

/**
 * @brief vertex reprensenting instantaneous position of the robot relative to itself
 * Trivial: in the graph it's always located at the origin with an orientation of 0 degrees, and
 * should always be connected to the vertex representing the current state.
 * 
 */
const vertexDescriptor movingVertex=0; 

class Planner{
    protected:

    public:



    /**
     * @brief Implements custom algorithm to search transition system for a plan
     * 
     * @param g the transitionSystem
     * @param src vertex from which the planning start
     * @param info execution info
     * @param finished will return true if the planning process reaches the goal
     * @return std::vector<vertexDescriptor> 
     */
    virtual std::vector<vertexDescriptor> plan(TransitionSystem& g, vertexDescriptor src, ExecutionInfo & info, bool *finished);

    /**
     * @brief calculates cumulative cost phi, add discount factor if in plan
     * 
     * @param er contains information on whether a Task has ended and with that cumulative cost (phi)
     * @param v the vertex that the Task corresponds to
     * @param p the plan
     * @return float 
     */
    static float evaluationFunction(EndedResult er, const vertexDescriptor &v, std::vector<vertexDescriptor>& p);

    /**
     * @brief Estimates cost (phi) of a state as a measure of:
     *      past cost (gamma): the position relative to an obstacle, if present
     *      future cost heuristic (chi): the position relative to a goal, if present
     * 
     * @param state the current state
     * @param start where the task started from
     * @param _goal the overaraching goal
     * @return EndedResult 
     */
    static EndedResult estimateCost(const State &state, b2Transform start, Task & _goal); //returns whether the controlGoal has ended and fills node with cost and error


};

/**
 * @brief Performs iterative deepening search over one unit of modular expansion (see below),
 * i.e. search over one branch stops when it reaches a DEFAULT task. Each DEFAULT Task represents the frontier.
 * The planner expands the search from the frontier with the lowest cost phi (A*-style search), which are logged into a priority queue
 * 
 * NB: maximum amount of nodes traversed to find a DEFAULT task is three from the start node, including the DEFAULT task
 * 
 *                            EXAMPLE MODULE

                    q2(LEFT)---q3(DEFAULT)
                   /   
                 q0 --- q1(DEFAULT)
                   \
                    q4(RIGHT)
                             \
                              q5(RIGHT)---q6(DEFAULT)      ->  maximum depth
 */
class HorizonStarPlanner:public Planner{
    protected:
    /**
     * @brief The depth-first search portion of iterative deepening will stop when it reaches a DEFAULT task
     * 
     */
    const Direction frontier_direction=DEFAULT;

    /**
     * @brief Finds the best path to add a frontier to (frontier found separately)
     * 
     * @param path the path to add to
     * @param add the frontier
     * @param paths all the paths
     * @param g the transition system
     */
    void path2add2(std::vector<std::vector<vertexDescriptor>>::reverse_iterator & path, const std::vector <vertexDescriptor> & add, std::vector<std::vector<vertexDescriptor>> &paths, TransitionSystem &g);

    /**
     * @brief Searches all paths for best path according to the lowest final cost phi
     * 
     * @param paths all the paths
     * @param goal the goal vertex (if known)
     * @param cv the current vertex
     * @param change whether the task needs to be changed
     * @param g the transition system
     * @return the plan!
     */
    std::vector <vertexDescriptor> best_path(const std::vector<std::vector<vertexDescriptor>>& paths, vertexDescriptor goal,  vertexDescriptor cv,  bool change, const TransitionSystem& g);


    /**
     * @brief Performs iterative deepening search to find the frontier
     * 
     * @param v source vertex: we find the frontier from here
     * @param g the transitionSystem
     * @param d 
     * @param  info
     * @return std::vector <Frontier> 
     */
    std::vector <Frontier> frontierVertices(vertexDescriptor v, TransitionSystem& g, ExecutionInfo & info); //returns the closest vertices to the start vertex which are reached by executing a task of the specified direction


    /**
     * @brief Adds frontier to priority queue
     * 
     * @param f the frontier
     * @param queue the priority queue
     * @param g the transition system
     * @param goal the vertex where the goal is reach
     */
    void addToPriorityQueue(const Frontier &f, std::vector<Frontier>& queue, TransitionSystem &g, vertexDescriptor goal=TransitionSystem::null_vertex());


public:

    std::vector<vertexDescriptor> plan(TransitionSystem& g, vertexDescriptor src, ExecutionInfo & info, bool * finished=NULL)override;

};

#endif