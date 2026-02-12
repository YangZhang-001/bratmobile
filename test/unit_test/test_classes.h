#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"
#include <string>
#include <numeric>

const std::string SYNTH_DATA_FOLDER="/synthetic/";

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

template <typename T, class P>
bool for_all(std::vector<T> vec, P predicate){
    for (auto &v:vec){
        if (!predicate(v)){
            return false;
        }
    }
    return true;
}

bool operator==( EndCriteria &ec1, EndCriteria& ec2){
    return ec1.angle==ec2.angle && ec1.distance==ec2.distance;

}





/**
 * @brief Predicate used to decide if an edge has been visited
 * 
 */
struct VisitedEdge{ 
	VisitedEdge()=default;
	VisitedEdge(TransitionSystem * ts, int _it):g(ts), iteration(_it){}

	bool operator()(const edgeDescriptor&e){
        bool result=(*g)[e].it_observed==iteration;
		return result;
	}

    int getIteration(){return iteration;}
	private:
	TransitionSystem *g=NULL;
    int iteration=0;
};

// //not sure why it doesn't filter the TS!
// typedef boost::filtered_graph<TransitionSystem, VisitedEdge> VisitedTransitionSystem;


class DebugConfigurator:public virtual AttentiveConfigurator{
    public:
    friend class HighLevelTestBase;
    friend class HighLevelInterruptBase;

    void unregister_tracker(){
        if (tracker){
            delete tracker;
        }
    }

    int n_edges(){return transitionSystem.m_edges.size();}

    int n_vertices(){return transitionSystem.m_vertices.size();}

    int n_visitedEdges(){
        int count=0;
        auto es=boost::edges(transitionSystem);
        for (auto ei=es.first; ei!=es.second; ei++){
            count+=transitionSystem[*ei].it_observed==iteration;
        }
        return count;
    }

    const std::vector <vertexDescriptor>& get_plan(){ return m_plan;}

    std::vector <vertexDescriptor>& get_plan_nConst(){ return m_plan;}

    bool plan_reaches_horizon();

    bool plan_reaches_goal();

    vertexDescriptor plan_end(){return m_plan[m_plan.size()-1];}

    b2Vec2 plan_end_b2Vec2(){return transitionSystem[plan_end()].endPose.p;}

    Task & getTask(){ //returns Task being executed
        return currentTask;
    }

    std::vector <BodyFeatures> & world_objects(){
        return worldBuilder->get_world_objects();
    }

    TransitionSystem & get_ts(){
        return transitionSystem;
    }

    TransitionSystem* get_ts_ptr(){
        return &transitionSystem;
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

    void vertex_set_phi(vertexDescriptor v, float f){
        transitionSystem[v].phi=f;
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
        return transitionSystem[v].options;
    }

    /**
     * @brief makes bodyfeatures
     * 
     */
    BodyFeatures bodyFeatures(float x, float y, float q, float hlength, float hwidth);

    void setTask(const Task& t){
        currentTask=t;
    }

    vertexDescriptor get_current_vertex(){
        return currentVertex;
    }
    
        /**
     * @brief Wrapper
     * 
     */
    void preExplore(){
        pre_explore();
    }

    int get_vertex_in_degree(vertexDescriptor v);

    int get_vertex_out_degree(vertexDescriptor v);


    int get_movingEdge_step(){
        return transitionSystem[movingEdge].step;
    }

    Logger * get_logger(){
        return logger;
    }

    WorldBuilder * get_worldbuilder(){
        return worldBuilder;
    }

    void getFeatures(const CoordinateContainer & cc){
        worldBuilder->set_world_objects(worldBuilder->getFeatures(cc, b2Transform_zero, WorldBuilder::PARTITION));

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

    /**
     * @brief Assign phi to state @param s
     * 
     * @param s 
     */
    void setPhi(State & s);

        /**
     * @brief Assign phi to all vertices
     * 
     */
    void setAllVisited();

    static Task generateGoalTask();

    static Disturbance generateGoal();

        /**
     * @brief Creates a vertex whose state starts and end at the origin. Not visited by default.
     * DOES NOT set pose
     * 
     * @param v0 
     * @return edgeDescriptor 
     */
    edgeDescriptor make_successful(vertexDescriptor v0=0, Direction direction=DEFAULT);

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


    void add_edge_withPoses(vertexDescriptor u, vertexDescriptor v);

    void addStepToEdge(edgeDescriptor e);

    void data2fp_emplace(Pointf p){
        data2fp.emplace(p);
    }
    
    const CoordinateContainer &get_data2fp(){return data2fp;}

    /**
     * @brief Wrapper around explore_plan
     * 
     * @param world 
     */
    void explorePlan(b2World & world){explore_plan(world);}

    GoalChanger * getGoalChanger()const{
        return goal_changer;
    }

};

class DebugB2B: public virtual DebugConfigurator, public virtual B2BConfigurator{
    protected:
    using B2BConfigurator::getDisturbance;
    using B2BConfigurator::closeVertex;
    using B2BConfigurator::splitTask;
    using B2BConfigurator::explorer;

    public:
    DebugB2B()=default;
    
    /**
     * @brief Also a mock class
     * 
     */
    class ClearVoyanceTest:public B2BConfigurator::ClearVoyance{
        public:
            int size(){return lookaheads.size();}

           // MOCK_METHOD(Disturbance, query, (vertexDescriptor v), (override));

    };

};

class LogicalCheckPlanTest:public DebugConfigurator, public testing::TestWithParam<std::tuple<vertexDescriptor, AffordanceIndex,Direction, bool>>{};

class DebugB2BTest: public virtual DebugB2B, public testing::Test{
};


class DebugB2BOptions: public virtual DebugB2B, public testing::Test{

    void SetUp()override{
        dummy_vertex(MOVING_VERTEX);
    }
    void TearDown()override{
        transitionSystem=TransitionSystem(1);
    }
};

class DebugB2BTestVertex:public DebugB2BTest, public testing::WithParamInterface<vertexDescriptor>{
    public:
    DebugB2BTestVertex(){}
    
};

class DebugDiscreteConf: public virtual DebugConfigurator, public virtual DiscreteConfigurator{
    Disturbance getDisturbance(TransitionSystem&g, vertexDescriptor v, b2World & world, const Direction & dir, const b2Transform& start)override{
        return DiscreteConfigurator::getDisturbance(g, v, world, dir, start);
    }

    Robot makeRobot(b2World & w, const Task & task)override{
        return DiscreteConfigurator::makeRobot(w, task);
    }

    float remainingSimulationTime(const Task *const t=NULL)override{
        return DiscreteConfigurator::remainingSimulationTime(t);
    }

    VertexMatch findMatch(State s, Direction dir=Direction::UNDEFINED, StateMatcher::MATCH_TYPE match_type=StateMatcher::_TRUE, StateDifference * _sd=NULL, vertexDescriptor src=TransitionSystem::null_vertex())override{
        return DiscreteConfigurator::findMatch(s, dir, match_type, _sd);
    }

    void transitionMatrix(vertexDescriptor v, Direction d, vertexDescriptor src) override{
        return DiscreteConfigurator::transitionMatrix(v, d, src);
    }

    bool closeVertex(std::set<vertexDescriptor> & closed, vertexDescriptor v)override{
        return DiscreteConfigurator::closeVertex(closed, v);
    }

    std::vector<Direction> partiallyExplorativeOptions(std::pair<bool, edgeDescriptor> ve)override{
        return DiscreteConfigurator::partiallyExplorativeOptions(ve);
    }
    
    bool shouldPartiallyExplore(const std::vector<edgeDescriptor>& oe, std::pair<bool, edgeDescriptor> ve)override{
        return DiscreteConfigurator::shouldPartiallyExplore(oe, ve);
    }

    void removeExploredTransitions(vertexDescriptor v)override{
        return DiscreteConfigurator::removeExploredTransitions(v);
    }

    bool canPropagate(vertexDescriptor v) override{
        return DiscreteConfigurator::canPropagate(v);
    }

    bool canReassignOutcome(vertexDescriptor v) override{
        return DiscreteConfigurator::canReassignOutcome(v);
    }

    bool propagateD(vertexDescriptor v1, vertexDescriptor v0, std::set<vertexDescriptor>*closed=NULL, StateMatcher::MATCH_TYPE match=StateMatcher::_FALSE)override{
        return DiscreteConfigurator::propagateD(v1, v0, closed, match);
    }

    float customSimulationStep(vertexDescriptor v)override{
        return DiscreteConfigurator::customSimulationStep();
    }


};


class WiseControllerTest: public Wise_Controller, public ::testing::Test{};

/**
 * @brief Gives Worlbuilder option to generate synthetic data
 * 
 */
class CreativeWorldBuilder: public WorldBuilder{
    public:

    /**
     * @brief 
     * 
     * @param width total width (x axis) of cul de sac
     * @param halfLength half length (y axis) of the cul the sac
     * @param shift 
     * @return std::vector <BodyFeatures> : the different panels making up the cul de sac
     *  
     *                              width
     *                             ===================         
     *                                                ||    l
     *                                                ||    e
     *                  ROBOT --->          x         ||    n
     *                        shift                   ||    g
     *                                                ||    t
     *                                                ||    h
     *                             ===================
     */
    static std::vector <BodyFeatures> makeCulDeSac(float width, float halfLength, b2Vec2 shift=b2Vec2(0,0));

    /**
     * @brief Makes a scenario where task splitting will fail
     * 
     *              ||
     * 
     *              ROBOT --->      || 
     *        
     *              ||    
     *          
     *                             x=0.3m   
     * 
     * @return std::vector <BodyFeatures> 
     */
    static std::vector <BodyFeatures> makeTricky(float dist =0.3);

    static std::vector <BodyFeatures> makeTrickyTrap(float dist =0.3);

    void addObject(const BodyFeatures& bf){world_objects.push_back(bf);}
};

class WorldBuilderTest: public CreativeWorldBuilder, public ::testing::Test{};

 /**
 * @brief Test fixture for testing high-level processes such as planning and state-space exploration
 * 
 * @param bool does plan have a target location
 * @param string the folder with the LIDAR scans
 */
class HighLevelTestBase: public testing::Test{
    protected:
    DebugConfigurator * configurator=NULL;
    Wise_Controller wc;
    ClosedLoop_Tracker tracker;
    DataInterface di;
    MotorInterface m;
    HorizonStarPlanner planner;
    

    int iteration=0;
    virtual void SetUp()override{
        configurator=new DebugConfigurator();
        init();
    }

    void TearDown()override{
        delete configurator;
    }
    /**
     * @brief From the parametrized values in string format @param valueParam (which are the values of this instance of the parametrized test), extracts the folder name
     */
    std::string parseFolder(std::string valueParam);

    /**
     * @brief Gets the iteration from @param valueParam (values of the parametrized test)
     */
    std::string parseIteration(std::string valueParam);

    /**
     * @brief Make logger that dumps in different directories depending on test case and system architecture, only for test fixtures
    */
    virtual Logger makeLogger();


        /**
     * @brief Make logger that dumps in different directories depending on test case and system architecture
    */
    virtual Logger makeLogger(const char * testInfo);


    /**
     * @brief Simulates execution by updating task state each scan
     * 
     * @param nScans how many scans for
     */
    void trackFor(int nScans);

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


};

class HighLevelTest: public virtual HighLevelTestBase , public testing::WithParamInterface<std::tuple<bool, std::string, int>>{
    public:
    HighLevelTest(){}

    bool has180Turn(std::vector<vertexDescriptor> plan){
        if (plan.empty()){
            return false;
        }
        for (int i=0; i<=plan.size(); i++){
            if (configurator->get_ts()[plan[i+1]].isTurning() && configurator->get_ts()[plan[i]].isTurning()){
                return true;
            }

        }
    }
};

class HighLevelTestB2B:  public HighLevelTest{ //Base , public testing::WithParamInterface<std::tuple<bool, std::string, int>>    
    public:
    HighLevelTestB2B(){};


    virtual void SetUp()override{
       configurator=new DebugB2B;
       init();
    }
    virtual void TearDown()override{
       delete configurator;
    }
};

class HighLevelTestDiscrete:  public HighLevelTest{ //Base , public testing::WithParamInterface<std::tuple<bool, std::string, int>>    
    public:
    HighLevelTestDiscrete(){};


    virtual void SetUp()override{
       configurator=new DebugDiscreteConf;
       init();
    }
    virtual void TearDown()override{
        delete configurator;
    }
};

/**
 * @brief For testing how the sysyem reacts when a plan s intrrupted
 * 
 */
class HighLevelInterruptBase: public HighLevelTestBase{
    protected:
        /**
     * @brief Tests plan vs a scenario with one single point representing an obstacle interrupting a task
     * 
     * @param folder
     * @param it iteration of data interface (determines which map will be read) - 0 reads map 1
     * @param taskOrder order of task in plan we want to interrupt. 0 is the current task
     * @param pt point that interrupts the plan
     * 
     */
    std::vector<vertexDescriptor> get_InterruptedPlan(std::string folder,int it, int taskOrder, Pointf *pt =NULL);

    Pointf generateInterruptingPoint(int taskOrder);

};

class HighLevelInterruptTest: public HighLevelInterruptBase, public testing::WithParamInterface<std::tuple<bool, std::string, int, int>>{};

class HighLevelInterruptTestTest: public HighLevelInterruptBase, public testing::WithParamInterface<Direction>{};

class ReactToNoiseTest: public HighLevelTestBase, public ::testing::WithParamInterface<std::tuple<bool, std::string, std::string, int>>{
    protected:
    std::pair<std::string, std::string> carveScenario(std::string valueParam);
    /**
     * @brief Make logger that dumps in different directories depending on test case and system architecture
    */
    Logger makeLogger(const char * testInfo)override;


};

class CLTrackerTest:public ClosedLoop_Tracker{
    public:
    void setDeltaTransform(b2Transform t){deltaTransform=t;}
    
    void set_tracked_disturbance(const Disturbance & d){
        tracked_disturbance=d;
    }

};

/**
 * @brief Fixture class for testing Configurator functions
 */
class ConfiguratorTest: public virtual DebugConfigurator, public testing::Test{ //, testing::TestWithParam<float>
protected:
    friend HighLevelTestBase;
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



    void set_Di(std::vector<vertexDescriptor> vec, const Disturbance& Di);

    void set_Dn(std::vector<vertexDescriptor> vec, const Disturbance& Dn);


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
class ConfiguratorTestGetGoal:public virtual ConfiguratorTest, public testing::WithParamInterface<std::tuple<b2Transform, Direction,Direction>>{
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

class ConfiguratorTestGetObstacle: public virtual ConfiguratorTestGetGoal{
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

class B2BTestGetGoal: public virtual DebugB2B, public virtual ConfiguratorTestGetGoal{}; 

class B2BTestGetObstacle: public virtual DebugB2B, public virtual ConfiguratorTestGetObstacle{}; 


class ConfiguratorEvaluationQueueManagerTest: public ConfiguratorTest, public testing::WithParamInterface<std::tuple<Direction, simResult::resultType >>{
    protected:
    std::vector<vertexDescriptor> evaluationQ;
    AttentiveConfigurator::EvaluationQueueManager eqm;
    vertexDescriptor v=MOVING_VERTEX;
    void SetUp(){
        transitionSystem[MOVING_VERTEX].direction=STOP;
    }

    void TearDown(){
        evaluationQ.clear();
        eqm.reset();
        transitionSystem=TransitionSystem(1);
    }
};

/**
 * @brief Testing if deadreckonign happens correctly
 */
class TestDeadReckoning:public testing::TestWithParam<std::tuple<Direction, Direction>>{};

class ReactiveConfTest: public ReactiveConfigurator, public ::testing::TestWithParam<Direction>{
    protected:
    float solution=BOX2DRANGE;
    void SetUp()override{
        if (GetParam()==DEFAULT){
            solution=simulationStep;
        }
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

std::vector <BodyFeatures> CreativeWorldBuilder::makeCulDeSac(float width, float halfLength, b2Vec2 shift){
    BodyFeatures front, Lside, Rside;
    front.pose.p=b2Vec2(width+shift.x, 0+shift.y);
    front.halfLength= halfLength-.05;
    front.halfWidth= 0.01; //width of the panel
    Lside=front, Rside=front;
    Lside.pose.p=b2Vec2(0+shift.x, halfLength+shift.y);
    Rside.pose.p=b2Vec2(0+shift.x, -halfLength-shift.y);
    Lside.pose.q.Set(M_PI_2);
    Rside.pose.q.Set(-M_PI_2);
    return std::vector <BodyFeatures>({front, Lside, Rside});

}

std::vector <BodyFeatures> CreativeWorldBuilder::makeTricky(float dist){
    BodyFeatures front, Lside, Rside, back;
    front.pose.p=b2Vec2(dist, 0);
    Lside.pose.p=b2Vec2(0, dist);
    Rside.pose.p=b2Vec2(0, -dist);
    return std::vector <BodyFeatures>({front, Lside, Rside});
}

std::vector <BodyFeatures> CreativeWorldBuilder::makeTrickyTrap(float dist){
    std::vector <BodyFeatures> result=CreativeWorldBuilder::makeTricky(dist);
    BodyFeatures trap;
    trap.pose.p.x=-dist;
    result.push_back(trap); //trap
    return result;
}





void HighLevelTestBase::init( const Task& goal){
    di.registerConfigurator(configurator);
    configurator->register_controller(&wc);
    configurator->register_tracker(&tracker);
    configurator->registerInterface(&m);
    configurator->setSimulationStep(ROBOT_HALFWIDTH*2);
    configurator->register_planner(&planner);
    configurator->init(goal);
    configurator->currentTask.set_change(true);

}


std::vector<vertexDescriptor> HighLevelTestBase::get_plan(std::string folder, int it){
    di.set_iteration(it);
    EXPECT_EQ(configurator->n_visitedEdges(), 0);
    if (folder!=SYNTH_DATA_FOLDER){
        di.set_folder(folder);
        di.newScanAvail();        
    }
    else{
        di.reset();
        configurator->data2fp.emplace(Pointf(0.5,0)); //one point
    }
    if (configurator->getIteration()>1){
        int visitedEdges=configurator->n_visitedEdges();
        EXPECT_LT(visitedEdges, configurator->n_edges());
        EXPECT_GT(visitedEdges, 0);
    }
    return configurator->get_plan();
}

std::vector<vertexDescriptor> HighLevelInterruptBase::get_InterruptedPlan(std::string folder,int it, int taskOrder, Pointf *pt){
    di.set_iteration(it);
    if (di.hasFolder()){
       // di.set_folder(folder);
        di.newScanAvail();
    }
    else{
        b2Vec2 pt2d(configurator->get_data2fp().begin()->x, configurator->get_data2fp().begin()->y);
        pt2d=b2Mul(configurator->getTask().getAction().getTransform(LIDAR_SAMPLING_RATE), pt2d);
        configurator->data2fp.erase(configurator->data2fp.begin());
        configurator->data2fp.emplace(Pointf(pt2d.x, pt2d.y));
    }
    Pointf pf=generateInterruptingPoint(taskOrder);
    if (pt!=NULL){
        *pt=pf;
    }
    configurator->data2fp_emplace(pf);
    configurator->Spawner();
    EXPECT_GT(configurator->n_visitedEdges(), 0);
    return configurator->get_plan();
}

Pointf HighLevelInterruptBase::generateInterruptingPoint(int taskOrder){
    vertexDescriptor vertexToInterrupt=configurator->get_current_vertex();
    if (taskOrder>-1){    //get task order length
        vertexToInterrupt=configurator->get_plan()[taskOrder];
    }
    b2Vec2 cornerFromCentroid(ROBOT_HALFWIDTH, ROBOT_HALFLENGTH);   
    b2Vec2 pt0(-(cornerFromCentroid.Length()-0.01), 0);
    b2Vec2 pt(0,0);
    if (configurator->vertex_get_direction(vertexToInterrupt)==DEFAULT){
        if (configurator->vertex_get_Di(vertexToInterrupt).getAffIndex()==NONE && taskOrder==-1){
            pt.x=BOX2DRANGE;
        }
        else{
            pt.x=configurator->vertex_get_endPose(vertexToInterrupt).p.x;
            pt.y=configurator->vertex_get_endPose(vertexToInterrupt).p.y;
        }
    }
    else{
        if (configurator->vertex_get_direction(vertexToInterrupt)==LEFT){
            pt0=b2Vec2(-cornerFromCentroid.x, -cornerFromCentroid.y); //bl
        }
        else if (configurator->vertex_get_direction(vertexToInterrupt)==RIGHT){
            pt0=b2Vec2(-cornerFromCentroid.x, cornerFromCentroid.y); //tl
        }
        pt=b2Mul(configurator->vertex_get_endPose(vertexToInterrupt), pt0);
    }
    return Pointf(pt.x, pt.y);
}
    


std::string HighLevelTestBase::parseFolder(std::string valueParam){
    int firstSlash=valueParam.find_first_of("/");
    valueParam.erase(valueParam.begin(), valueParam.begin()+firstSlash);
    int lastSlash=valueParam.find_last_of("/");
    valueParam.erase(valueParam.begin()+lastSlash,valueParam.end());
    return valueParam;
}

std::string HighLevelTestBase::parseIteration(std::string valueParam){
    int lastSpace=valueParam.find_last_of(" ");
    valueParam.erase(valueParam.begin(), valueParam.begin()+lastSpace+1);
    int lastParenthesis=valueParam.find_last_of(")");
    valueParam.erase(valueParam.begin()+lastParenthesis,valueParam.end());
    return valueParam; 
}

std::pair<std::string, std::string> ReactToNoiseTest::carveScenario(std::string valueParam){
    valueParam.erase(valueParam.begin());
    int firstSlash=valueParam.find_first_of("/");
    int lastSlash=valueParam.find_last_of("/");
    std::string str1(valueParam.begin(), valueParam.begin()+firstSlash);
    std::string str2(valueParam.begin()+lastSlash+1, valueParam.end());
    return std::pair<std::string, std::string>(str1, str2);
}


void HighLevelTestBase::trackFor(int iteration){
    for (int i=0;i<iteration-1; i++){ //simulate execution
    if (configurator->getIteration()>1){
        TrackingResult trackingResult= tracker.track(configurator->getTask(), configurator->data2fp, configurator->world_objects() );
        //EXPECT_FALSE(deltaPose==b2Transform_zero);
        configurator->update_graph(configurator->get_ts(), trackingResult);
    }
    configurator->change_task();
    configurator->estimate_current_vertex();    
    configurator->addIteration();
    EXPECT_GT(configurator->get_current_vertices().size(), 0);
    EXPECT_NE(configurator->get_current_vertices()[0], 0);
    if (di.hasFolder()){

        di.newScanAvail(false); //do not do plan
    }
    else{
        b2Vec2 pt(configurator->data2fp.begin()->x, configurator->data2fp.begin()->y);
        pt=b2Mul(configurator->getTask().getAction().getTransform(LIDAR_SAMPLING_RATE), pt);
        configurator->data2fp.erase(configurator->data2fp.begin());
        configurator->data2fp.emplace(Pointf(pt.x, pt.y));
    }
    configurator->getFeatures(configurator->data2fp);
    configurator->preExplore();
    EXPECT_EQ(configurator->n_visitedEdges(), 0);
    EXPECT_GT(configurator->get_vertex_out_degree(0), 0);
}

}

Logger HighLevelTestBase::makeLogger(){
    std::string dumpFolder="benchmark", systemArchDir=dumpFolder+Logger::getSystemArchitecture();
    std::string testCaseDir=::testing::UnitTest::GetInstance()->current_test_info()->name();
    return Logger(testCaseDir.c_str(), systemArchDir.c_str());
}

Logger HighLevelTestBase::makeLogger(const char * testInfo){
    std::string dumpFolder="benchmark", systemArchDir=dumpFolder+Logger::getSystemArchitecture();
    std::string addOn, dash("_"),  testCaseDir=::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string scenario;
    if (std::size_t index=testCaseDir.find_first_of("/"); index!=std::string::npos){
        addOn=parseIteration(std::string(testInfo));
        addOn=dash+addOn;
        testCaseDir.erase(testCaseDir.begin()+index, testCaseDir.end());
        scenario=parseFolder(std::string(testInfo))+addOn;
    }
    // testCaseDir=testCaseDir;
    return Logger(testCaseDir.c_str(), systemArchDir.c_str(), scenario.c_str());
}

Logger ReactToNoiseTest::makeLogger(const char * testInfo){
    std::string dumpFolder="benchmark", systemArchDir=dumpFolder+Logger::getSystemArchitecture();
    std::string addOn, dash("_"),  testCaseDir=::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::string bothScenarios, scenario;
    if (std::size_t index=testCaseDir.find_first_of("/"); index!=std::string::npos){
        addOn=parseIteration(std::string(testInfo));
        addOn=dash+addOn;
        testCaseDir.erase(testCaseDir.begin()+index, testCaseDir.end());
        bothScenarios=parseFolder(std::string(testInfo));
        std::pair<std::string, std::string> separateScenarios=carveScenario(bothScenarios);
        scenario=separateScenarios.first+ separateScenarios.second+addOn;
    }
    // testCaseDir=testCaseDir;
    return Logger(testCaseDir.c_str(), systemArchDir.c_str(), scenario.c_str());
}

edgeDescriptor DebugConfigurator::make_successful(vertexDescriptor v0, Direction direction){
    auto v1=boost::add_vertex(transitionSystem);
    auto e=boost::add_edge(v0, v1, transitionSystem);
    transitionSystem[v1].direction=direction;
    transitionSystem[e.first].step=1;
    transitionSystem[e.first].it_observed=iteration;
    return e.first;
}

edgeDescriptor DebugConfigurator::make_v1_crashed( vertexDescriptor v0, b2Transform start, b2Transform end, b2Transform Dn){
    edgeDescriptor e=make_successful(v0);
    vertexDescriptor v1=e.m_target;
    transitionSystem[v1].outcome=simResult::crashed;
    transitionSystem[v1].start=start; //start
    transitionSystem[v1].endPose=end;//pose
    transitionSystem[v1].Dn=Disturbance(AVOID, Dn.p,Dn.q.GetAngle());
    transitionSystem[v1].Dn.validate();
    transitionSystem[e].it_observed=iteration;
    return e;
}

void DebugConfigurator::make_module(vertexDescriptor mv){
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


BodyFeatures DebugConfigurator::bodyFeatures(float x, float y, float q, float hlength, float hwidth){
    BodyFeatures bf;
    bf.pose.p.x=x;
    bf.pose.p.y=y;
    bf.pose.q.Set(q);
    bf.halfLength=hlength;
    bf.halfWidth=hwidth;
    bf.attention=true;
    return bf;
}

void DebugConfigurator::add_edge_withPoses(vertexDescriptor u, vertexDescriptor v){
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

void DebugConfigurator::addStepToEdge(edgeDescriptor e){
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
    return 0;
}

void ConfiguratorTestTransitionMatrix::planIsDirection(Direction direction){
    auto oe=gt::outEdges(transitionSystem, currentVertex, direction);
    if (oe.empty()){
        return;
    }
    m_plan={oe[0].m_target};

}

void DebugConfigurator::setAllVisited(){
    auto vs=boost::vertices(transitionSystem);
    for (auto vi=vs.first; vi!=vs.second; vi++){
        setPhi(transitionSystem[*vi]);
    }
}

void DebugConfigurator::setPhi(State & s){
    s.phi=estimateCost(s, s.start, s.direction, controlGoal).cost;   
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