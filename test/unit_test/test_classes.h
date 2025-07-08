#ifndef TEST_CLASSES_H
#define TEST_CLASSES_H

#include <gtest/gtest.h>
#include "../callbacks.h"
#include <string>

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

    const std::vector <vertexDescriptor>& get_plan(){ return plan;}

    bool plan_reaches_horizon();

    bool plan_reaches_goal();

    vertexDescriptor plan_end(){return plan[plan.size()-1];}

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
        plan.clear();
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
        plan=_p;
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
     * @brief Creates a vertex whose state starts and end at the origin
     * 
     * @param v0 
     * @return edgeDescriptor 
     */
    edgeDescriptor make_successful(vertexDescriptor v0=0);
    /**
     * @brief returns an edge connecting vertex v0 to a vertex pointing to a crashed state
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
     */
    void make_module(vertexDescriptor mv=0);
public:

    /**
     * @brief makes bodyfeatures
     * 
     */
    BodyFeatures bodyFeatures(float x, float y, float q, float hlength, float hwidth);

    void add_edge_withStep(vertexDescriptor u, vertexDescriptor v);

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
    }

    void TearDown(){
        transitionSystem.clear();
    }
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
    if (n_vertices()==1){
        dummy_vertex(mv);
    }
    mv=currentVertex;
    for (int i=0; i<6; i++){
        boost::add_vertex(transitionSystem);
    }
    transitionSystem[mv+1].direction=DEFAULT;
    transitionSystem[mv+3].direction=DEFAULT;
    transitionSystem[mv+5].direction=DEFAULT;
    transitionSystem[mv+2].direction=LEFT;
    transitionSystem[mv+4].direction=RIGHT;

    transitionSystem[mv+2].endPose.q.Set(M_PI_2);
    transitionSystem[mv+4].endPose.q.Set(-M_PI_2);
    b2Transform distance=b2Transform(b2Vec2(0.5, 0), b2Rot(0));
    transitionSystem[mv+3].start=transitionSystem[mv+2].endPose;
    transitionSystem[mv+3].endPose=b2MulT(transitionSystem[mv+2].endPose, distance);
    transitionSystem[mv+5].start=transitionSystem[mv+4].endPose;
    transitionSystem[mv+5].endPose=b2MulT(transitionSystem[mv+4].endPose, distance);

    add_edge_withStep(mv,mv+1);
    add_edge_withStep(mv,mv+2);
    add_edge_withStep(mv,mv+4);
    add_edge_withStep(mv+2,mv+3);
    add_edge_withStep(mv+4,mv+5);

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

void ConfiguratorTest::add_edge_withStep(vertexDescriptor u, vertexDescriptor v){
    auto e=boost::add_edge(u, v, transitionSystem);
    Task::Action a;
    a.init(transitionSystem[v].direction);
    transitionSystem[e.first].step=Controller::motor_step(a, transitionSystem[v].distance());
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



#endif