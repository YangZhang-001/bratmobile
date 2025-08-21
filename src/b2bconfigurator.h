#ifndef B2BCONFIGURATOR_H
#define B2BCONFIGURATOR_H
#include "attentive.h"

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

class B2BConfigurator : public virtual AttentiveConfigurator {
public:
    B2BConfigurator() = default;

    B2BConfigurator(Task & task) {
        init(task);
    }

protected:

    bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v) override;

    //std::vector<Direction> partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve) override;

    std::vector <vertexDescriptor> splitTask(vertexDescriptor v, Direction d, vertexDescriptor src=TransitionSystem::null_vertex()) override;

    //virtual void backtrack(std::vector <vertexDescriptor>& evaluation_q, std::vector <vertexDescriptor>&priority_q, std::set<vertexDescriptor>& closed, std::vector <vertexDescriptor>& plan_prov, vertexDescriptor module_src=MOVING_VERTEX, vertexDescriptor startRecycle=MOVING_VERTEX)override;

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
        if (frontiers.size()<2){ //only default explored
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

    bool add(vertexDescriptor v, const Disturbance& d);

    /**
     * @brief Gets first disturbance predicted with a vertex
     * 
     * @param v vertex
     * @return Disturbance 
     */
    Disturbance query(vertexDescriptor v);

    /**
     * @brief Pops the vertex query (first of the disturbances)
     */
    void pop(vertexDescriptor v);

    void reset() {lookaheads.clear();}
    protected:

    std::vector<DisturbanceLookahead> lookaheads;
}clearvoyance;

void addOptionsInHindsight(vertexDescriptor v, vertexDescriptor v0, vertexDescriptor v1, ClearVoyance & clearvoyance);
};

#endif