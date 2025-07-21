#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"
#include <string>
#include <numeric>


/**
 * @brief Setting up ostream operator for use with GTest
 * 
 * @param os ostream
 * @param t box2d 2dtransform
 * @return std::ostream& 
 */
std::ostream& operator<<(std::ostream& os, const b2Transform& t){
    os << "b2Transform(pos=("
       << t.p.x << ", " << t.p.y << "), rot=("
       << t.q.c << ", " << t.q.s << "))";
    return os;
}

class DebugConfigurator:public AttentiveConfigurator{
    public:
    friend class HighLevelTest;
    int n_edges(){return transitionSystem.m_edges.size();}

    int n_vertices(){return transitionSystem.m_vertices.size();}

    const std::vector <vertexDescriptor>& get_plan(){ return m_plan;}

    bool plan_reaches_horizon();

    bool plan_reaches_goal();

    vertexDescriptor plan_end(){return m_plan[m_plan.size()-1];}

    b2Vec2 plan_end_b2Vec2(){return transitionSystem[plan_end()].endPose.p;}

    Task & getTask(){ //returns Task being executed
        return currentTask;
    }

    std::vector <BodyFeatures> & world_objects(){
        return worldBuilder.get_world_objects();
    }

    TransitionSystem & get_ts(){
        return transitionSystem;
    }

    Task & getGoal(){
        return controlGoal;
    }

    void set_data2fp(const CoordinateContainer &data){
        data2fp=data;
    }

    int data_size(){
        return data2fp.size();
    }

    void clear_plan(){
        m_plan.clear();
    }


    void vertex_set_state(vertexDescriptor v, const State &s){
        transitionSystem[v]=s;
    }

    void vertex_set_endPose(vertexDescriptor v, b2Transform t){
        transitionSystem[v].endPose=t;
    }

    void vertex_set_start(vertexDescriptor v, b2Transform t){
        transitionSystem[v].start=t;
    }

    void vertex_set_direction(vertexDescriptor v, Direction d){
        transitionSystem[v].direction=d;
    }

    void vertex_set_outcome(vertexDescriptor v, simResult::resultType r){
        transitionSystem[v].outcome=r;
    }

    void vertex_set_Di(vertexDescriptor v, const Disturbance & d){
        transitionSystem[v].Di=d;
    }

    void vertex_set_Dn(vertexDescriptor v, const Disturbance & d){
        transitionSystem[v].Dn=d;
    }

    void vertex_set_options(vertexDescriptor v, const std::vector <Direction> & d){
        transitionSystem[v].options=d;
    }

    void vertex_options_push_back(vertexDescriptor v, Direction d){
        transitionSystem[v].options.push_back(d);
    }

    const State & vertex_get_state(vertexDescriptor v){
        return transitionSystem[v];
    }

    b2Transform vertex_get_endPose(vertexDescriptor v){
        return transitionSystem[v].endPose;
    }

    b2Transform vertex_get_start(vertexDescriptor v){
        return transitionSystem[v].start;
    }

    Direction vertex_get_direction(vertexDescriptor v){
        return transitionSystem[v].direction;
    }

    simResult::resultType vertex_get_outcome(vertexDescriptor v){
        return transitionSystem[v].outcome;
    }

    const Disturbance & vertex_get_Di(vertexDescriptor v){
        return transitionSystem[v].Di;
    }

    const Disturbance & vertex_get_Dn(vertexDescriptor v){
        return transitionSystem[v].Dn;
    }

    const std::vector <Direction>& vertex_get_options(vertexDescriptor v){
        transitionSystem[v].options;
    }
    

    vertexDescriptor get_current_vertex(){
        return currentVertex;
    }
    

    int get_vertex_in_degree(vertexDescriptor v);

    int get_vertex_out_degree(vertexDescriptor v);

    /**
     * @brief Wrapper
     * 
     */
    void preExplore(){
        pre_explore();
    }

    int get_movingEdge_step(){
        return transitionSystem[movingEdge].step;
    }

    Logger * get_logger(){
        return logger;
    }

    WorldBuilder * get_worldbuilder(){
        return &worldBuilder;
    }

    void getFeatures(const CoordinateContainer & cc){
        worldBuilder.set_world_objects(worldBuilder.getFeatures(cc, b2Transform_zero, WorldBuilder::PARTITION));

    }

    void set_edge_step(vertexDescriptor u, vertexDescriptor v, int step){
        auto e=boost::edge(u, v, transitionSystem);
        if (e.second){
            transitionSystem[e.first].step=step;
        }
    }

    void set_plan(std::vector<vertexDescriptor> _p){
        m_plan=_p;
    }

    void set_running(bool b){
        running=b;
    }

    void set_current_v(vertexDescriptor v){
        currentVertex=v;
        current_vertices={v};
    }

    std::vector<vertexDescriptor>& get_current_vertices(){
        return current_vertices;
    }

    static Task generateGoalTask();

    static Disturbance generateGoal();
    

};


 /**
 * @brief Test fixture for testing high-level processes such as planning and state-space exploration
 * 
 * @param bool does plan have a target location
 * @param string the folder with the LIDAR scans
 */
class HighLevelTest: public testing::Test, public testing::WithParamInterface<std::tuple<bool, std::string, int>>{
    protected:
    DebugConfigurator * configurator=NULL;
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    LIDAR_In ci;
    DataInterface di;
    Motor_Out m;
    HorizonStarPlanner planner;
    
    int iteration=0;
    void SetUp()override{
        std::cout<<"setup"<<std::endl;
        configurator=new DebugConfigurator();
        init();
        std::cout<<"teardown"<<std::endl;
    }

    void TearDown()override{
        delete configurator;
    }
    /**
     * @brief From the parametrized values in string format @param valueParam , extracts the folder name
     * 
     * @return const char* 
     */
    std::string parseFolder(std::string valueParam);

    /**
     * @brief Make logger that dumps in different directories depending on test case and system architecture
    */
    Logger makeLogger(const char * testInfo="");

    /**
     * @brief Simulates execution by updating task state each scan
     * 
     * @param nScans how many scans for
     */
    void iterateFor(int nScans);

    /**
     * @brief Initialises Fixture
     * 
     * @param goal overarching goal
     */
    void init( const Task& goal=Task());
    /**
     * @brief Tests planning
     * 
     * @param folder a folder containing LIDAR scans names "map%04i.dat"
     * @param it iteration of data interface (determines which map will be read) - 0 reads map 1
     * 
     */
    std::vector<vertexDescriptor> get_plan(std::string folder, int it=0);
    public:

    HighLevelTest(){}


};



/**
 * @brief Fixture class for testing Configurator functions
 */
class ConfiguratorTest: public DebugConfigurator, public testing::Test{ //, testing::TestWithParam<float>
protected:
    /**
     * @brief Allows to set parameters manually from the Configurator
     * 
     */
    class Manual_WiseController:public Wise_Controller{
        public:
        /**
         * @brief Manually set the distance of Di(disturbance_q) to goal
         * 
         * @param goal goal disturbance
         */
        void set_Di_to_goal(const Disturbance & goal){
            _D_to_goal=b2MulT(disturbance_q.pose(), goal.pose());
        }
    };

    /**
     * @brief Creates a vertex whose state starts and end at the origin. Not visited by default
     * 
     * @param v0 
     * @return edgeDescriptor 
     */
    edgeDescriptor make_successful(vertexDescriptor v0=0);
    /**
     * @brief returns an edge connecting vertex v0 to a vertex pointing to a crashed state. Not visited by default
     * 
     */
    edgeDescriptor make_v1_crashed( vertexDescriptor v0=0, b2Transform start=b2Transform_zero, b2Transform end=b2Transform_zero, b2Transform Dn=b2Transform_inf);

    /**
     * @brief Makes a basic expansion module,  disturbances not set. Module looks like this
     *               
     *              v2(LEFT)---v3(DEFAULT)
                   /   
                 v0 --- q1(DEFAULT)
                   \
                    v4(RIGHT) --- v5(DEFAULT)
     * 
     * Not visited by default
     */
    void make_module(vertexDescriptor mv=0);

    /**
     * @brief Assign phi to all vertices
     * 
     */
    void setAllVisited();

    /**
     * @brief Assign phi to state @param s
     * 
     * @param s 
     */
    void setPhi(State & s);

    void set_Di(std::vector<vertexDescriptor> vec, const Disturbance& Di);

    void set_Dn(std::vector<vertexDescriptor> vec, const Disturbance& Dn);

public:

    /**
     * @brief makes bodyfeatures
     * 
     */
    BodyFeatures bodyFeatures(float x, float y, float q, float hlength, float hwidth);

    void add_edge_withPoses(vertexDescriptor u, vertexDescriptor v);

    void addStepToEdge(edgeDescriptor e);

};


/**
 * @brief Configurator fixture parametrised for b2Transforms
 * 
 */
class ConfiguratorTest2DT:public ConfiguratorTest, public testing::WithParamInterface<b2Transform>{

};

/**
 * @brief Configurator parameters are 2 2dtransforms
 * 
 */
class ConfiguratorTest32DT:public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, b2Transform,b2Transform>>{
    protected:
    ConfiguratorTest32DT(){
        register_tracker(new ClosedLoop_Tracker);
    }

    ~ConfiguratorTest32DT(){
        delete tracker;
    }
    
    public:
        void SetUp(){
        transitionSystem=TransitionSystem(1);
        setAllVisited();
    }

    void TearDown(){
        transitionSystem.clear();
    }
};

class ConfiguratorBacktrackTest: public ConfiguratorTest32DT{
    protected:
};


class ConfiguratorTestTransitionMatrix: public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, Direction, simResult::resultType>>{
protected:
    /**
     * @brief number of expected options
     * 
     * @param dir direction of current vertex
     * @param o outcome of current vertex task
     * @param d goal disturbance
     * @return int 
     */
    int expectedOptions(Direction dir, simResult::resultType o, Disturbance d);

    /**
     * @brief sets plan to the vertex that has a certain direction
     * 
     */
    void planIsDirection(Direction direction);

    void SetUp()override{
        transitionSystem=TransitionSystem(1);
        currentVertex=MOVING_VERTEX;
    }

    void TearDown()override{
        transitionSystem.clear();
    }

};


class ConfiguratorTakeBool:public ConfiguratorTest, public testing::WithParamInterface<bool>{
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
        transitionSystem=TransitionSystem(1);
    }

    /**
     * @brief Expands vertex @param v with a maximum depth of 1
     */
    void shallowExpand(vertexDescriptor v);

};

/**
 * @brief int is the number of vertices we want crashed
 * 
 */
class ConfiguratorPlannerHybrid: public ConfiguratorTakeBool, public HorizonStarPlanner, public ::testing::WithParamInterface<std::tuple<int, Direction, simResult::resultType>>{
    protected:
    std::vector<vertexDescriptor> withDirection(Direction d);

    void assignOutcome();

    int n_successful(std::vector<vertexDescriptor> vec);

};

/**
 * @brief Parameters: robot position, previous task direction, current task direction
 * 
 */
class ConfiguratorTestGetGoal:public ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, Direction,Direction>>{
    protected:
    /**
     * @brief Sets up vertex for testing using the parameters
     * 
     * @param v 
     */
    void vertex_setup(vertexDescriptor v, const Disturbance & Di, const Disturbance &Dn=Disturbance());

    void SetUp(){
        Disturbance goal(PURSUE, b2Vec2(1.0, 0));
        init(Task(goal, UNDEFINED));
        data2fp.emplace(Pointf(0.55, 0)); //make point corresponding to obstacle
        dummy_vertex(MOVING_VERTEX);
    }

    void TearDown(){
        controlGoal=Task();
        boost::remove_vertex(currentVertex, transitionSystem);
    }
};

class ConfiguratorTestGetObstacle: public ConfiguratorTestGetGoal{
    protected:
    void SetUp(){
        Disturbance goal(PURSUE, b2Vec2(1.0, 0));
        init(Task(goal, UNDEFINED));
        data2fp.emplace(Pointf(0.55, 0)); //make point corresponding to obstacle
        dummy_vertex(MOVING_VERTEX);
    }

    void TearDown(){
        controlGoal=Task();
        boost::remove_vertex(currentVertex, transitionSystem);
    }
};



////////////////////////////////////////////////////////////////////////

int DebugConfigurator::get_vertex_in_degree(vertexDescriptor v){
    return boost::in_degree(v, transitionSystem);
}

int DebugConfigurator::get_vertex_out_degree(vertexDescriptor v){
    return boost::out_degree(v, transitionSystem);
}



bool DebugConfigurator::plan_reaches_horizon(){
    return fabs(plan_end_b2Vec2().Length()-BOX2DRANGE)<0.02;
}

bool DebugConfigurator::plan_reaches_goal(){
    return (plan_end_b2Vec2()-controlGoal.get_disturbance().pose().p).Length()<0.02;
}

Task DebugConfigurator::generateGoalTask(){
    return Task(generateGoal(), UNDEFINED);   
}

Disturbance DebugConfigurator::generateGoal(){
    return Disturbance(PURSUE, b2Vec2(1.0,0));
}


void HighLevelTest::init( const Task& goal){
    di.registerInterface(&ci);
    configurator->register_controller(&wc);
    configurator->register_tracker(&tracker);
    configurator->registerInterface(&ci, &m);
    configurator->setSimulationStep(ROBOT_HALFWIDTH*2);
    configurator->register_planner(&planner);
    configurator->init(goal);
    configurator->currentTask.set_change(true);

}


std::vector<vertexDescriptor> HighLevelTest::get_plan(std::string folder, int it){
    di.set_iteration(it);
    di.set_folder(folder);
    di.newScanAvail();
    configurator->data2fp= ci.data2fp;
    configurator->Spawner();
    return configurator->get_plan();
}

std::string HighLevelTest::parseFolder(std::string valueParam){
    int firstSlash=valueParam.find_first_of("/");
    valueParam.erase(valueParam.begin(), valueParam.begin()+firstSlash);
    int lastSlash=valueParam.find_last_of("/");
    valueParam.erase(valueParam.begin()+lastSlash,valueParam.end());
    return valueParam;
}

void HighLevelTest::iterateFor(int iteration){
    for (int i=0;i<iteration-1; i++){ //simulate execution
    if (configurator->getIteration()>1){
        b2Transform deltaPose= tracker.track(configurator->getTask(), ci.data2fp, configurator->world_objects() );
        //EXPECT_FALSE(deltaPose==b2Transform_zero);
        configurator->update_graph(configurator->get_ts(), deltaPose);
    }
    configurator->change_task();
    configurator->estimate_current_vertex();    
    configurator->addIteration();
    EXPECT_GT(configurator->get_current_vertices().size(), 0);
    EXPECT_NE(configurator->get_current_vertices()[0], 0);
    di.newScanAvail();
    configurator->getFeatures(ci.data2fp);
    configurator->preExplore();
    EXPECT_GT(configurator->get_vertex_out_degree(0), 0);
}

}

Logger HighLevelTest::makeLogger(const char * testInfo){
    std::string dumpFolder="benchmark", systemArchDir=dumpFolder+Logger::getSystemArchitecture();
    std::string addOn, dash("_"),  testCaseDir=::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string scenario;
    if (std::size_t index=testCaseDir.find_first_of("/"); index!=std::string::npos){
        addOn.append(testCaseDir.begin()+index+1, testCaseDir.end());
        addOn=dash+addOn;
        testCaseDir.erase(testCaseDir.begin()+index, testCaseDir.end());
        scenario=parseFolder(std::string(testInfo))+addOn;
    }
    // testCaseDir=testCaseDir;
    return Logger(testCaseDir.c_str(), systemArchDir.c_str(), scenario.c_str());
}

edgeDescriptor ConfiguratorTest::make_successful(vertexDescriptor v0){
    auto v1=boost::add_vertex(transitionSystem);
    auto e=boost::add_edge(v0, v1, transitionSystem);
    transitionSystem[v1].direction=DEFAULT;
    transitionSystem[e.first].step=1;
    transitionSystem[e.first].it_observed=iteration;
    return e.first;
}

edgeDescriptor ConfiguratorTest::make_v1_crashed( vertexDescriptor v0, b2Transform start, b2Transform end, b2Transform Dn){
    edgeDescriptor e=make_successful(v0);
    vertexDescriptor v1=e.m_target;
    transitionSystem[v1].outcome=simResult::crashed;
    transitionSystem[v1].start=start; //start
    transitionSystem[v1].endPose=end;//pose
    transitionSystem[v1].Dn=Disturbance(AVOID, Dn.p,Dn.q.GetAngle());
    transitionSystem[e].it_observed=iteration;
    return e;
}

void ConfiguratorTest::make_module(vertexDescriptor mv){
    //mv=currentVertex;
    std::vector<vertexDescriptor>new_vertices;
    for (int i=0; i<5; i++){
        new_vertices.push_back(boost::add_vertex(transitionSystem));
    }
    // vertexDescriptor nv=n_vertices()-1;
    transitionSystem[new_vertices[0]].direction=DEFAULT;
    transitionSystem[new_vertices[2]].direction=DEFAULT;
    transitionSystem[new_vertices[4]].direction=DEFAULT;
    transitionSystem[new_vertices[1]].direction=LEFT;
    transitionSystem[new_vertices[3]].direction=RIGHT;

    add_edge_withPoses(mv,new_vertices[0]);
    add_edge_withPoses(mv,new_vertices[1]);
    add_edge_withPoses(mv,new_vertices[3]);
    add_edge_withPoses(new_vertices[1],new_vertices[2]);
    add_edge_withPoses(new_vertices[3],new_vertices[4]);

}

BodyFeatures ConfiguratorTest::bodyFeatures(float x, float y, float q, float hlength, float hwidth){
    BodyFeatures bf;
    bf.pose.p.x=x;
    bf.pose.p.y=y;
    bf.pose.q.Set(q);
    bf.halfLength=hlength;
    bf.halfWidth=hwidth;
    bf.attention=true;
    return bf;
}

void ConfiguratorTest::add_edge_withPoses(vertexDescriptor u, vertexDescriptor v){
    transitionSystem[v].start=transitionSystem[u].endPose;
    b2Transform distance=b2Transform_zero;
    switch (transitionSystem[v].direction){
        case DEFAULT:
            distance.p.x=.5;
        break;
        case LEFT:
            distance.q.Set(M_PI_2);
        break;
        case RIGHT:
            distance.q.Set(-M_PI_2);
        break;
        default: break;
    }
    transitionSystem[v].endPose=b2Mul(distance, transitionSystem[v].start);
    auto e=boost::add_edge(u, v, transitionSystem);
    addStepToEdge(e.first);
    transitionSystem[e.first].it_observed=iteration;
}

void ConfiguratorTest::addStepToEdge(edgeDescriptor e){
    Task::Action a;
    Direction direction=transitionSystem[e.m_target].direction;
    a.init(direction);
    transitionSystem[e].step=Controller::motor_step(a, transitionSystem[e.m_target].distance());

    
}


int ConfiguratorTestTransitionMatrix::expectedOptions(Direction dir, simResult::resultType o, Disturbance d){
    if (o==simResult::safeForNow){
        if(dir==DEFAULT || dir==STOP){
            return 2;
        }
    }
    else if (o==simResult::successful){
        if (dir==LEFT || dir==RIGHT){
            if (d.getPosition().x<0){
                return 2;
            }
            else{
                return 1;
            }
        }
        else{
           if (d.isValid() && d.getPosition().y !=0 ){
                return 3;
            }
            else{
                return 1;
            }
        }

    }
    else if (o==simResult::crashed){
        return 0;
    }
}

void ConfiguratorTestTransitionMatrix::planIsDirection(Direction direction){
    auto oe=gt::outEdges(transitionSystem, currentVertex, direction);
    if (oe.empty()){
        return;
    }
    m_plan={oe[0].m_target};

}

void ConfiguratorTest::setAllVisited(){
    auto vs=boost::vertices(transitionSystem);
    for (auto vi=vs.first; vi!=vs.second; vi++){
        setPhi(transitionSystem[*vi]);
    }
}

void ConfiguratorTest::setPhi(State & s){
    s.phi=Planner::estimateCost(s, s.start, s.direction, controlGoal).cost;   
}

void ConfiguratorTest::set_Di(std::vector<vertexDescriptor> vec, const Disturbance& Di){
    for (vertexDescriptor v:vec){
        vertex_set_Di(v, Di);
    }
}

void ConfiguratorTest::set_Dn(std::vector<vertexDescriptor> vec, const Disturbance& Dn){
        for (vertexDescriptor v:vec){
        vertex_set_Dn(v, Dn);
    }
}


void ConfiguratorTakeBool::make_ts(std::vector <vertexDescriptor>& avoid, std::vector<vertexDescriptor>& desiredPlan, bool hasGoal){
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

void ConfiguratorTakeBool::shallowExpand(vertexDescriptor v){
    vertexDescriptor v1, v2, v3;
    Disturbance disturbance;
    transitionSystem[v].options={DEFAULT, LEFT, RIGHT};
    Edge e;
    e.it_observed=iteration;
    add_vertex_now(v, v1, disturbance, e, false);
    if (GetParam()){
        add_vertex_now(v, v2, disturbance, e, false);
        add_vertex_now(v, v3, disturbance, e, false);
    }
}

void ConfiguratorTestGetGoal::vertex_setup(vertexDescriptor v, const Disturbance & Di, const Disturbance &Dn){
    transitionSystem[v].direction=std::get<1>(GetParam());
    transitionSystem[v].endPose=std::get<0>(GetParam());
    vertex_options_push_back(v, std::get<2>(GetParam()));
    transitionSystem[v].Di=Di; 
    transitionSystem[v].Di.validate();
    transitionSystem[v].Dn=Dn; 
    if (Dn.getAffIndex()!=NONE){
        transitionSystem[v].Dn.validate();  
    }
}
#endif