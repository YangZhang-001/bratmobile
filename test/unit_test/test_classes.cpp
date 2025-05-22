#include "test_classes.h"
Configurator_Test::Configurator_Test(Task * goal){
    configurator=Configurator(goal);
    configurator.register_controller(&wc);
    configurator.register_tracker(&tracker);
    configurator.registerInterface(&ci, &m);
}

std::vector<vertexDescriptor> Configurator_Test::get_plan(char * folder){
    DataInterface di(&ci);
    di.folder=folder;
    di.newScanAvail();
    configurator.data2fp= CoordinateContainer(ci->data2fp);
    configurator.Spawner();
    return configurator.plan;
}
