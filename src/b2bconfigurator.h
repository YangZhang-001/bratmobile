#ifndef B2BCONFIGURATOR_H
#define B2BCONFIGURATOR_H
#include "attentive.h"
/**
 * @brief Predicate assessing whether a frontier is a crashed state
 * 
 */
struct FrontierCrashed{
    FrontierCrashed() = default;

    FrontierCrashed(TransitionSystem & ts, Direction d) : transitionSystem(ts), direction(d) {}

    bool operator()(const Frontier & f) const {
        if (f.connecting.empty()) return false;
        return transitionSystem[f.connecting[0]].direction==direction && transitionSystem[f.frontier].outcome==simResult::crashed;
    }
    private:
    TransitionSystem & transitionSystem;
    Direction direction=UNDEFINED;
};

/**
 * @brief B2BConfigurator is a class that extends the AttentiveConfigurator to eliminate the Task splitting in fixed sized steps
 * but finds waypoints by propagating disturbances back in time and across branches of the transitionSystem
 * 
 */
class B2BConfigurator : public virtual AttentiveConfigurator {
public:
    B2BConfigurator() = default;

    B2BConfigurator(Task & task) {
        init(task);
    }

protected:

    /**
     * @brief Closes a vertex but the maximum out edges number for a default state to have is 5 instead of 3
     * 
     * @param closed 
     * @param v 
     */
    bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v) override;

    //std::vector<Direction> partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve) override;

    /**
     * @brief Doesn't split the task, just returns the vertex of the task at hand; if Task fails or 
     * it's currently executing, it returns the source vertex for the task aswell
     * 
     * @param v vertex of the task
     * @param d direction of the task that are allowed to split (not used)
     * @param src source of the task
     * @return std::vector <vertexDescriptor> 
     */
    std::vector <vertexDescriptor> splitTask(vertexDescriptor v, Direction d, vertexDescriptor src=TransitionSystem::null_vertex()) override;

    /**
     * @brief Same as AttentiveConfigurator::backtrack, but also adds vertex to priority queue if the vertex is in the clearvoyance
     * 
     * @param evaluation_q 
     * @param priority_q 
     * @param closed 
     * @param plan_prov 
     * @param module_src 
     * @param startRecycle 
     */
   virtual void backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src=MOVING_VERTEX, vertexDescriptor startRecycle=MOVING_VERTEX)override;

    /**
     * @brief Returns true if attention window overlap with Di
     * 
     * @param Di previous state's Di
     * @param q previous state
     * @param world box2d world
     * @param focus disturbance to keep in focus to make the attetnion window (e.g. goal)
     */
    bool attentionWindowOverlaps(const Disturbance & Di, const State & q, b2World & world, const Disturbance & focus );

    /**
     * @brief Counts the number of visited edges
     * @param es the edges
     * @return int 
     */
    int visitedEdgeCount(const std::vector <edgeDescriptor>& es);

    int minimumEdgesForClearvoyance(Direction direction);

   // bool canGoToClearVoyance(const std::vector <edgeDescriptor> &oe, Direction direction);
    /**
     * @brief Uses clearvoyance to get the disturbance for a vertex if needed
     * 
     * @param g 
     * @param v 
     * @param world 
     * @param dir 
     * @param start 
     * @return Disturbance 
     */
    virtual Disturbance getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start) override;

    //virtual std::vector <vertexDescriptor> task_vertices(vertexDescriptor v, std::pair<bool, edgeDescriptor>* ep=NULL);

    /**
     * @brief Looks at the frontier to unlock transitions from a vertex
     * 
     * @tparam P predicate
     * @param v the vertex
     * @param predicate the assignment criteria if the frontier has less than 2 vertices
     * @return std::vector <Direction> 
     */
    template <typename P>
    std::vector <Direction> transitionInHindsight(vertexDescriptor v, P predicate){
        std::vector <Direction> result;
        ExecutionInfo info=package_info();
        std::vector <Frontier> frontiers=frontierVertices(v, transitionSystem, info);
        if (frontiers.size()<2){ //only  explored
           // result={DEFAULT, LEFT, RIGHT};
            //erase_from_vector(result, transitionSystem[ve.second.m_target].direction);
           // return result;
           result= predicate(v);
        }
        else if (frontiers.size()<4){ //left right explored
            auto fLeft= std::find_if(frontiers.begin(), frontiers.end(), FrontierCrashed(transitionSystem, LEFT));
            auto fRight= std::find_if(frontiers.begin(), frontiers.end(), FrontierCrashed(transitionSystem, RIGHT));
            if (fLeft!=frontiers.end()){
                result.push_back(DEFAULT);
            } 
            if (fRight!=frontiers.end()){
                result.push_back(DEFAULT);
            } 
        }
        return result;
    }

    //     /**
    // *Combines edges K and jump function: represents possible transitions out of a state
    // *@param v the vertex to which transitions are being assigned
    // *@param d state direction (redundant)
    // *@param src source vertex of state
    // */
    // virtual void transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src)override; 

    // virtual void removeExploredTransitions(vertexDescriptor v);

/**
 * @brief Constructs transition system using a Box2D simulation combined with an A* graph
 * expansion algorithm
 * 
 * @param v starting vertex
 * @param g the transition system
 * @param w box2d world
 * @return std::vector<vertexDescriptor> a plan, if recycled from previous knowledge
 */
virtual std::vector<vertexDescriptor> explorer(vertexDescriptor v, TransitionSystem&g, b2World &w)override; //evaluates only after DEFAULT, internal one step lookahead

virtual simResult simulate(Task t, b2World & world, vertexDescriptor v0); 

// /**
//  * @brief Overload of makeRobot, uses a disturbance which may be the goal of the hindsight disturbance to make the sensor
//  * 
//  * @param t 
//  * @param world 
//  * @param focus disturbance focus of attention (used for making the sensor)
//  * @return Robot 
//  */
// virtual Robot makeRobot( b2World & world, const b2Transform& start, const Disturbance & focus);
/**
 * @brief Stores disturbance lookaheads for alternative DEFAULT tasks (where the disturbance is backpropagated)
 * 
 */
class ClearVoyance{
    public:
    struct DisturbanceLookahead {
        std::vector<Disturbance> disturbances;
        vertexDescriptor source;

        DisturbanceLookahead()=default;

        DisturbanceLookahead( vertexDescriptor v, const Disturbance & d) : source(v) {
            disturbances.push_back(d);
        }

    };

    virtual bool add(vertexDescriptor v, const Disturbance& d);

    /**
     * @brief Gets first disturbance predicted with a vertex
     * 
     * @param v vertex
     * @return Disturbance 
     */
    virtual Disturbance query(vertexDescriptor v);

    /**
     * @brief Pops the first disturbance off of the vector associated with the queried vertex
     */
    void pop(vertexDescriptor v);

    void reset() {lookaheads.clear();}

    std::vector <DisturbanceLookahead> getLookaheads() const {
        return lookaheads;
    }
    protected:

    std::vector<DisturbanceLookahead> lookaheads;
}clearvoyance;

/**
 * @brief If the frontier is a default state and its source is a turning task, it adds an additional default option in hindsight if the state crashed
 * 
 * @param v source vertex (options are added to this vertex in hindsight)
 * @param v0 connecting vertex (has to be LEFT or RIGHT task for DEFAULT option to be added)
 * @param v1 frontier state (has to be DEFAULT and crashed if option is to be added)
 * @param clearvoyance 
 */
void addOptionsInHindsight(vertexDescriptor v, vertexDescriptor v0, vertexDescriptor v1, ClearVoyance & clearvoyance);


};

#endif