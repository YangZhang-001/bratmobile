#ifndef TASK_CONTROLLER_H
#define TASK_CONTROLLER_H

#include "graphTools.h"
#include "task.h"

/**
 * @brief Callback interface class for implementing custom strategies to switch between tasks to be executed
 * 
 */
class Controller
{
  protected:
    Disturbance
        disturbance_q; //disturbance being counteracted as in the cognitive map
    b2Transform _D_to_goal = b2Transform_zero;
    /**
     * @brief Get the current vertex 
     * 
     * @param cv vector of vertices making up the current task
     */
    vertexDescriptor
    get_current_vertex (const std::vector<vertexDescriptor> &cv)
    {
        if (cv.size () > 0)
        {
            return cv[0];
        }
        return 0;
    }
    Task stopTask (const Task &controlGoal);

  public:
    Controller () = default;

    /**
     * @brief Virtual function used to choose the next task
     * 
     * @param currentTask the current task
     * @param controlGoal the overarching goal task
     * @param g the cognitive map
     * @param current_vertices a container with the cognitive map vertices representing the current Task (more than one if Task has been split)
     * @param plan the plan
     */
    virtual Task next_task (Task currentTask, const Task &controlGoal,
                            const TransitionSystem &g,
                            std::vector<vertexDescriptor> &current_vertices,
                            std::vector<vertexDescriptor> &plan)
        = 0;

    /**
     * @brief Assigns fixed duration (in amount of motor callbacks)
     * 
     * @param a the action of the task
     * @param distance the distance travelled in a task, default is robot length
     * @return int 
     */
    static int motor_step (Task::Action a, float distance = 0.27);

    /**
     * @brief Returns the disturbance being counteracted as in the cognitive map
     */
    Disturbance get_disturbance () { return disturbance_q; }

    void set_disturbance (const Disturbance &d) { disturbance_q = d; }

    /**
     * @brief returns the distance between the disturbance_q and the goal
     * this is used to maintain constant ratios during tracking
     * 
     * @return b2Transform 
     */
    b2Transform disturbance_to_goal () { return _D_to_goal; }
};

/**
 * @brief Chooses tasks based on a plan
 * 
 */
class Wise_Controller : public Controller
{
  public:
    Wise_Controller () = default;

    Task next_task (Task currentTask, const Task &controlGoal,
                    const TransitionSystem &g,
                    std::vector<vertexDescriptor> &current_vertices,
                    std::vector<vertexDescriptor> &plan);

    /**
     * @brief returns last vertex of the task starting at plan[0]
     * 
     * @param g the cognitive map
     * @param plan the plan
     * @return int iterator to the next vertex not belonging to the task to be executed immediately
     */
    int to_task_end (const TransitionSystem &g,
                     std::vector<vertexDescriptor> &plan);

    /**
 * @brief Creates a new task by interpreting the states in a plan. Tasks split in several states are treated like one task,
 *  and Tasks terminating in collisions are morphed into tasks aimed at reaching a certain distance from an obstacle
 * 
 * @param p 
 * @param g 
 * @param end_it 
 * @param controlGoal 
 * @param currentTask 
 * @param current_vertices 
 * @return Task 
 */
    Task
    task_to_execute (const std::vector<vertexDescriptor> &p,
                     const TransitionSystem &g, int end_it,
                     const Task &controlGoal, Task &currentTask,
                     const std::vector<vertexDescriptor> &current_vertices);
};

/**
  * @brief Chooses the next task reactively (Braitenberg controller)
  * 
  */
class Reactive_Controller : public Controller
{
  public:
    Reactive_Controller () = default;

    virtual Task next_task (Task currentTask, const Task &controlGoal,
                            const TransitionSystem &g,
                            std::vector<vertexDescriptor> &current_vertices,
                            std::vector<vertexDescriptor> &plan);
};

class OpenLoopController : public Controller
{
    Task next_task (Task currentTask, const Task &controlGoal,
                    const TransitionSystem &g,
                    std::vector<vertexDescriptor> &current_vertices,
                    std::vector<vertexDescriptor> &plan);
};
#endif