#ifndef PLANNER_H
#define PLANNER_H

#include "graphTools.h"

namespace Planner{
    //find path to add frontier (add) to
    void path2add2(std::vector<std::vector<vertexDescriptor>>::reverse_iterator &, const std::vector <vertexDescriptor> & , std::vector<std::vector<vertexDescriptor>> &, TransitionSystem &);

    std::vector <vertexDescriptor> best_path(const std::vector<std::vector<vertexDescriptor>>&, const vertexDescriptor& goal,  const vertexDescriptor&, const bool &, const TransitionSystem&);

};

#endif