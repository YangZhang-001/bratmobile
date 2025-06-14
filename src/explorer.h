#ifndef EXPLORER_H
#define EXPLORER_H

#include "planner.h"
#include "worldbuilder.h"

class Explorer{
protected:
WorldBuilder worldBuilder;
public:

virtual void set_up()=0;

virtual void explore()=0;

std::vector <BodyFeatures>& world_objects(){
    return worldBuilder.get_world_objects();
}

void world_objects(const std::vector <BodyFeatures>& wo){
    worldBuilder.set_world_objects(wo);
}


};

class HeuristcExploration:public Explorer{
public:

void set_up(TransitionSystem &, std::vector<vertexDescriptor>)


void explore();
};


#endif