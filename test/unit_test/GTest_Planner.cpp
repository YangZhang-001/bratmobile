#include "test_classes.h"

/**
 * @brief int is the number of vertices we want crashed
 * 
 */
class ConfiguratorTestPlanner: public ConfiguratorTest, public HorizonStarPlanner, public ::testing::WithParamInterface<std::tuple<int, Direction, simResult::resultType>>{
    protected:
    std::vector<vertexDescriptor> withDirection(Direction d);

    void assignOutcome();

    int n_successful(std::vector<vertexDescriptor> vec);

};

std::vector<vertexDescriptor> ConfiguratorTestPlanner::withDirection(Direction d){
    std::vector<vertexDescriptor> result;
    for (int i=1; i<n_vertices(); i++){
        if (transitionSystem[vertexDescriptor(i)].direction==d){
            result.push_back(i);
        }
   }
   return result;
}

int ConfiguratorTestPlanner::n_successful(std::vector<vertexDescriptor> vec){
    int count=0;
    for (vertexDescriptor v:vec){
        if (vertex_get_outcome(v)==simResult::successful){
            count++;
        }
    }
    return count;
}


void ConfiguratorTestPlanner::assignOutcome(){
    Direction direction=std::get<1>(GetParam());
    int n_assign=std::get<0>(GetParam());
    std::vector<vertexDescriptor> with_direction=withDirection(direction);
    simResult::resultType outcome=std::get<2>(GetParam());
    for (int i=0; i<with_direction.size(); i++){
        if (i<=n_assign){
            transitionSystem[with_direction[i]].outcome=outcome;
        }
    }
}


// TEST_P(ConfiguratorTestPlanner, frontierVertices){
//     dummy_vertex(MOVING_VERTEX);
//     make_module(currentVertex);
//     setAllVisited();
//     assignOutcome();
//     transitionSystem[currentEdge].overrideZeroSteps=true;
//     int expected=n_successful(withDirection(DEFAULT));
//     ExecutionInfo info=package_info();
//     auto frontier=frontierVertices(MOVING_VERTEX, transitionSystem, info);    
//     EXPECT_EQ(frontier.size(), expected);
    
// }

INSTANTIATE_TEST_CASE_P(FrontierGalore, ConfiguratorTestPlanner, testing::Combine(::testing::Values(0, 1, 2, 3), 
                                                                ::testing::Values(LEFT, RIGHT, DEFAULT), 
                                                                ::testing::Values(simResult::crashed, simResult::successful, simResult::safeForNow)));