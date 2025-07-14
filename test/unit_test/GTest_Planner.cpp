#include "test_classes.h"


class ConfiguratorTestPlanner:public ConfiguratorTest, public testing::WithParamInterface<bool>{
    public:

/**
 * @brief makes transition system looking like this
 * 
 *      *             
                     v3(LEFT)---v4(DEFAULT)                      v13(LEFT)*---v14(DEFAULT)*
                    /                                           /
                   v1 --- v2(DEFAULT)       v8(LEFT)*---v9(DEFAULT)* --- v12(DEFAULT)
                    \                       /                   \
                    v5(RIGHT)* --- v6(DEFAULT)* --- v7(DEFAULT)    v15(RIGHT) --- v16(DEFAULT)  
                                            \
                                              v10(RIGHT) --- v11(DEFAULT)        
                                                       
                                                    

        *                  
 * 
 * @param avoid vertices whose Di is an obstacle to avoid
 * @param desiredPlan 
 */

    void make_ts(std::vector <vertexDescriptor>& avoid, std::vector<vertexDescriptor>& desiredPlan, bool haGoal);

    void SetUp()override{
        register_planner(new HorizonStarPlanner);
    }
    void TearDown()override{
        delete planner;
    }

};
 
TEST_P(ConfiguratorTestPlanner, RecyclePlan){
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4};
    vertexDescriptor task_start=DUMMY;
    make_ts(avoid, desiredPlan, GetParam());    
    State s=transitionSystem[2];
    b2Transform shift=b2Mul(transitionSystem[DUMMY].endPose, transitionSystem[currentVertex].endPose);
    b2Transform shift_start=b2Transform_zero;
    math::applyAffineTrans(shift, transitionSystem);
    EXPECT_EQ(transitionSystem[currentVertex].endPose, b2Transform_zero);
    iteration=100;
    resetPhi();
    VertexMatch vm(StateMatcher::ABSTRACT, 2);
    auto edge =boost::add_edge(currentVertex, 2, transitionSystem);
    bool recycled=recycle_plan(currentVertex, currentVertex, task_start, vm.first, shift_start, s.start, edge, m_plan, s.direction);
    EXPECT_TRUE(recycled);
    EXPECT_EQ(m_plan, desiredPlan);
}

INSTANTIATE_TEST_CASE_P(GoalOrNot, ConfiguratorTestPlanner, testing::Bool());

/**
 * @brief int is the number of vertices we want crashed
 * 
 */
class ConfiguratorPlannerHybrid: public ConfiguratorTestPlanner, public HorizonStarPlanner, public ::testing::WithParamInterface<std::tuple<int, Direction, simResult::resultType>>{
    protected:
    std::vector<vertexDescriptor> withDirection(Direction d);

    void assignOutcome();

    int n_successful(std::vector<vertexDescriptor> vec);

};



//class HorizonStarPlannerTest: public HorizonStarPlanner, public ::testing::Test{};

TEST_F(ConfiguratorPlannerHybrid,pathToAddTo){
    HorizonStarPlanner horizonPlanner;
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4}, add;
    std::vector<std::vector<vertexDescriptor>> paths;
    paths.emplace_back(std::vector<vertexDescriptor>({14, 1, 3, 4}));
    vertexDescriptor task_start=DUMMY;
    make_ts(avoid, desiredPlan, true); 
    std::vector<std::vector<vertexDescriptor>>::reverse_iterator path=paths.rbegin();
    add={5, 6};
    path2add2(path, add, paths, transitionSystem);
    EXPECT_EQ(paths.size(), 2);
    EXPECT_EQ(paths[1][paths[1].size()-1], 1);
    EXPECT_EQ(paths[1][paths[1].size()-2], 14);
    EXPECT_EQ(*path, paths[1]);
    EXPECT_TRUE(boost::edge(*(path->rbegin()), add[0], transitionSystem).second);
    
}


TEST_P(ConfiguratorTestPlanner, startRecycle){
    std::vector<vertexDescriptor> avoid={3,5},desiredPlan={1,3,4}, add;
    std::vector<std::vector<vertexDescriptor>> paths;
    paths.emplace_back(std::vector<vertexDescriptor>({14, 1, 3, 4}));
    vertexDescriptor solution=DUMMY;
    make_ts(avoid, desiredPlan, true); 
    currentVertex=*desiredPlan.rbegin();
    if (!GetParam()){
        transitionSystem[2].outcome=simResult::successful;
        solution=currentVertex;
    }
    EXPECT_EQ(getRecyclingStart(currentVertex,2), solution);
    
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

// INSTANTIATE_TEST_CASE_P(FrontierGalore, ConfiguratorTestPlanner, testing::Combine(::testing::Values(0, 1, 2, 3), 
//                                                                 ::testing::Values(LEFT, RIGHT, DEFAULT), 
//                                                                 ::testing::Values(simResult::crashed, simResult::successful, simResult::safeForNow)));

void ConfiguratorTestPlanner::make_ts(std::vector <vertexDescriptor>& avoid, std::vector<vertexDescriptor>& desiredPlan, bool hasGoal){
    addIteration();
    b2Vec2 d_position(1,0);
    Disturbance obstacle(AVOID, b2Vec2(.6,0)), goal(PURSUE, d_position);
    if (hasGoal){
        init(Task(goal,DEFAULT));
    }
    dummy_vertex(MOVING_VERTEX);
    make_module(currentVertex);
    currentVertex=n_vertices()-1; //6, if no goal
    if (hasGoal){
        make_module(6);
        make_module(9);
        std::vector<vertexDescriptor> all(n_vertices()-1), safe(n_vertices()-7);
        std::iota(all.begin(), all.end(), 1);
        std::iota(safe.begin(), safe.end(), 7);
        avoid.push_back(4);
        avoid.push_back(6);
        set_Di(avoid, obstacle);
        set_Di(safe, goal);
        vertex_set_Di(2, goal);
        desiredPlan={1,5, 6, 8, 9, 13, 14};
        currentVertex=14;
    }
    else{
        transitionSystem[4].endPose.p.y=1;
        transitionSystem[6].endPose.p.y=-1;

    }
    vertex_set_Dn(2, obstacle);
    vertex_set_outcome(2,simResult::crashed);
    currentTask.setMotorStep(0);
    currentTask.set_change(1);
    }




std::vector<vertexDescriptor> ConfiguratorPlannerHybrid::withDirection(Direction d){
    std::vector<vertexDescriptor> result;
    for (int i=1; i<n_vertices(); i++){
        if (transitionSystem[vertexDescriptor(i)].direction==d){
            result.push_back(i);
        }
   }
   return result;
}

int ConfiguratorPlannerHybrid::n_successful(std::vector<vertexDescriptor> vec){
    int count=0;
    for (vertexDescriptor v:vec){
        if (vertex_get_outcome(v)==simResult::successful){
            count++;
        }
    }
    return count;
}


// void ConfiguratorPlannerHybrid::assignOutcome(){
//     Direction direction=std::get<1>(GetParam());
//     int n_assign=std::get<0>(GetParam());
//     std::vector<vertexDescriptor> with_direction=withDirection(direction);
//     simResult::resultType outcome=std::get<2>(GetParam());
//     for (int i=0; i<with_direction.size(); i++){
//         if (i<=n_assign){
//             transitionSystem[with_direction[i]].outcome=outcome;
//         }
//     }
// }