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

    B2BConfigurator(Task & task) : AttentiveConfigurator(task) {}

protected:

    bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v) override;

    std::vector<Direction> partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve) override;

    std::vector <vertexDescriptor> splitTask(vertexDescriptor v, Direction d, vertexDescriptor src=TransitionSystem::null_vertex()) override;

    virtual Disturbance getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start) override;
};

#endif