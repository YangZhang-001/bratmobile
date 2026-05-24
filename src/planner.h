#ifndef PLANNER_H
#define PLANNER_H

#include "graphTools.h"
#include "task.h"

/** \file */

/**
 * @brief Contains the frontier (first) and the connecting vertices
 * 
 */
struct Frontier
{
    vertexDescriptor frontier = TransitionSystem::null_vertex ();
    std::vector<vertexDescriptor> connecting;

    Frontier () = default;

    Frontier (vertexDescriptor _f, const std::vector<vertexDescriptor> &_c)
        : frontier (_f), connecting (_c)
    {
    }

    bool operator== (const Frontier &f) const
    {
        return frontier == f.frontier && connecting == f.connecting;
    }
};

/**
 * @brief Predicate which compares evaluation functions in State-Frontier pairs
 * 
 */
struct ComparePhi
{

    ComparePhi () {}

    bool operator() (const std::pair<State *, Frontier> &p1,
                     const std::pair<State *, Frontier> &p2) const
    {
        return (*p1.first).phi < (*p2.first).phi;
    }
};

/**
 * @brief Information about what the configurator is doing (current Task, current vertex), what it wants to do (overarching goal), and whether it has done it before (been, goal vertex) 
 * 
 */
class ExecutionInfo
{
  private:
    vertexDescriptor m_currentVertex = 0,
                     m_goalVertex = TransitionSystem::null_vertex ();
    Task m_currentTask;
    Task m_overarchingGoal;
    bool
        m_been; //has a plan been made to fulfill this overarching goal before?
    std::vector<vertexDescriptor> m_plan;

  protected:
    friend class Configurator;

    void been (bool b) { m_been = b; }

    void goalVertex (vertexDescriptor gv) { m_goalVertex = gv; }

  public:
    ExecutionInfo () {}

    ExecutionInfo (vertexDescriptor _cv, vertexDescriptor _goal, Task &_ct,
                   Task &_gt, bool _been, std::vector<vertexDescriptor> _plan)
    {
        m_currentVertex = _cv;
        m_goalVertex = _goal;
        m_currentTask = _ct;
        m_overarchingGoal = _gt;
        m_been = _been;
        m_plan = _plan;
    }
    void overarchingGoal (const Task &og) { m_overarchingGoal = og; }

    vertexDescriptor currentVertex () const { return m_currentVertex; }

    vertexDescriptor goalVertex () const { return m_goalVertex; }

    Task &currentTask () { return m_currentTask; }

    Task &overarchingGoal () { return m_overarchingGoal; }

    bool been () const { return m_been; }

    std::vector<vertexDescriptor> plan () const { return m_plan; }
};

/**
 * @brief Performs iterative deepening search to find the frontier
 * 
 * @param v source vertex: we find the frontier from here
 * @param g the transitionSystem
 * @param d 
 * @param  info
 * @return std::vector <Frontier> 
 */
std::vector<Frontier> frontierVertices (
    vertexDescriptor v, TransitionSystem &g,
    ExecutionInfo &
        info); //returns the closest vertices to the start vertex which are reached by executing a task of the specified direction

/**
 * @brief calculates cumulative cost phi, add discount factor if in plan
 * 
 * @param er contains information on whether a Task has ended and with that cumulative cost (phi)
 * @param v the vertex that the Task corresponds to
 * @param p the plan
 * @return float 
 */
float evaluationFunction (EndedResult er, const vertexDescriptor &v,
                          std::vector<vertexDescriptor> &p);

/**
 * @brief Provides a breakdown of the cost function into its components: past cost (gamma): the position relative to an obstacle, if present
 *      future cost heuristic (chi): the position relative to a goal, if present, and whether the Task @param _goal has ended
 * 
 * @param state the current state
 * @param start where the task started from
 * @param d task direction
 * @param _goal the overaraching goal
 * @return EndedResult 
 */
EndedResult estimateCost (
    const State &state, b2Transform start, Direction d,
    Task &
        _goal); //returns whether the controlGoal has ended and fills node with cost and error

/**
 * @brief Searches the transition system and extracts a plan
 * 
 */
class Planner
{
  protected:
  public:
    Planner () = default;

    /**
     * @brief Implements custom algorithm to search transition system for a plan
     * 
     * @param g the transitionSystem
     * @param src vertex from which the planning start
     * @param info execution info
     * @param finished will return true if the planning process reaches the goal
     * @return std::vector<vertexDescriptor> 
     */
    virtual std::vector<vertexDescriptor>
    plan (TransitionSystem g, vertexDescriptor src, ExecutionInfo &info,
          bool *finished)
        = 0;
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
class HorizonStarPlanner : public Planner
{
  protected:
    /**
     * @brief Finds the best path to add a frontier to (frontier found separately)
     * 
     * @param path the path to add to
     * @param add the frontier
     * @param paths all the paths
     * @param g the transition system
     */
    void path2add2 (
        std::vector<std::vector<vertexDescriptor> >::reverse_iterator &path,
        const std::vector<vertexDescriptor> &add,
        std::vector<std::vector<vertexDescriptor> > &paths,
        TransitionSystem &g);

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
    std::vector<vertexDescriptor>
    best_path (const std::vector<std::vector<vertexDescriptor> > &paths,
               vertexDescriptor goal, vertexDescriptor cv, bool change,
               const TransitionSystem &g);

    /**
     * @brief Adds frontier to priority queue
     * 
     * @param f the frontier
     * @param queue the priority queue
     * @param g the transition system
     * @param goal the vertex where the goal is reach
     */
    void addToPriorityQueue (const Frontier &f, std::vector<Frontier> &queue,
                             TransitionSystem &g,
                             const std::set<vertexDescriptor> &closed,
                             vertexDescriptor goal
                             = TransitionSystem::null_vertex ());

    // struct CostMap{
    //     std::map<vertexDescriptor, float> map;

    //     void init(TransitionSystem & g){
    //         auto vs=boost::vertices(g);
    //         for (auto vi=vs.first; vi!=vs.first; vi++){
    //             if (g[*vi].visited()){
    //                 map.emplace(std::make_pair(*vi,g[*vi].phi));
    //             }
    //         }
    //     }
    // }costMap;

  public:
    std::vector<vertexDescriptor> plan (TransitionSystem g,
                                        vertexDescriptor src,
                                        ExecutionInfo &info,
                                        bool *finished = NULL) override;
};

/**
 * @brief Does not extract a plan
 * 
 */
class NoPlanner : public Planner
{

    std::vector<vertexDescriptor> plan (TransitionSystem g,
                                        vertexDescriptor src,
                                        ExecutionInfo &info,
                                        bool *finished = NULL) override
    {
        return std::vector<vertexDescriptor> ();
    }
};

#endif