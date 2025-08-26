#include "../../custom_robot.h"
#include "ObjectPackagePubSubTypes.h"
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>

#ifndef PUBLISHER_CPP
#define PUBLISHER_CPP

class ObjectPackagePublisher
{
private:

    DomainParticipant* participant_ = nullptr;

    Publisher* publisher_ = nullptr;

    Topic* topic_ = nullptr;

    DataWriter* writer_ = nullptr;

    TypeSupport type_;

    class PubListener : public DataWriterListener
    {
    public:

        PubListener()
            : matched_(0)
        {
        }

        ~PubListener() override
        {
        }

        void on_publication_matched(
                DataWriter*,
                const PublicationMatchedStatus& info) override
        {
            if (info.current_count_change == 1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher matched." << std::endl;
            }
            else if (info.current_count_change == -1)
            {
                matched_ = info.total_count;
                std::cout << "Publisher unmatched." << std::endl;
            }
            else
            {
                std::cout << info.current_count_change
                        << " is not a valid value for PublicationMatchedStatus current count change." << std::endl;
            }
        }

        std::atomic_int matched_;

    } listener_;

public:

    ObjectPackagePublisher() : type_(new ObjectPackagePubSubType()) {}

    virtual ~ObjectPackagePublisher()
    {
        if (writer_ != nullptr)
        {
            publisher_->delete_datawriter(writer_);
        }
        if (publisher_ != nullptr)
        {
            participant_->delete_publisher(publisher_);
        }
        if (topic_ != nullptr)
        {
            participant_->delete_topic(topic_);
        }
        DomainParticipantFactory::get_instance()->delete_participant(participant_);
    }

    //!Initialize the publisher
    bool init()
    {
        DomainParticipantQos participantQos;
        participantQos.name("Participant_publisher");
        participant_ = DomainParticipantFactory::get_instance()->create_participant(0, participantQos);

        if (participant_ == nullptr)
        {
            return false;
        }

        // Register the Type
        type_.register_type(participant_);

        // Create the publications Topic
	// !! Important that this matches with the name of message defined in ObjectPackage.idl !!
        topic_ = participant_->create_topic("ObjectPackageTopic", "ObjectPackage", TOPIC_QOS_DEFAULT);

        if (topic_ == nullptr)
        {
            return false;
        }

        // Create the Publisher
        publisher_ = participant_->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);

        if (publisher_ == nullptr)
        {
            return false;
        }

        // Create the DataWriter
        writer_ = publisher_->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &listener_);

        if (writer_ == nullptr)
        {
            return false;
        }
        return true;
    }

    //!Send a publication
    bool publish(ObjectPackage& object)
    {
        if (listener_.matched_ > 0)
        {
            writer_->write(&object);
            return true;
        }
        return false;
    }

};
#endif


class AffordanceSetter{
    AffordanceIndex affordance=NONE;
    public:
    AffordanceSetter(){
        std::cout<<"ENTER AFFORDANCE, CAPITALISED:"<<std::endl;
        std::string str;
        std::cin >>str;
        init(str);
    }

    AffordanceSetter(std::string str){
        init(str);
    }
    
    void init(std::string text){
	if (text== "AVOID") affordance= AVOID;
	else if (text=="PURSUE") affordance= PURSUE;
	else if (text== "NONE") affordance= NONE;
    else{
        std::cout<<"WHAT?? VALID AFFORDANCE PLEASE"<<std::endl;
        std::cin >>text;
        return init(text); 
        }    
    }

    AffordanceIndex getAffIndex(){return affordance;}
};

class DirectionSetter{
    protected:
    Direction direction=DEFAULT;
    public:
    DirectionSetter(){
        std::cout<<"ENTER DIRECTION, CAPITALISED:"<<std::endl;
        std::string str;
        std::cin >>str;
        std::cin.ignore();   // eat the leftover '\n'
        std::cin.get();
        init(str);
    }

    DirectionSetter(std::string str){
        init(str);
    }
    void init(std::string text){
	if (text== "LEFT") direction= LEFT;
	else if (text=="RIGHT") direction= RIGHT;
	else if (text== "DEFAULT") direction= DEFAULT;
	else{
        std::cout<<"WHAT?? VALID DIRECTION PLEASE"<<std::endl;
        std::cin >>text;
        std::cin.ignore();   // eat the leftover '\n'
        return init(text);
    }
    }

    Direction getDirection(){
        return direction;
    }
};


class QtTracker:public ClosedLoop_Tracker{
    protected:
    Disturbance Di=Disturbance();
    ObjectPackagePublisher mypub;
    public:
    void setDisturbance(const Disturbance&d){
        Di=d;
    }

    void on_new_reading(Task *goal)override{
        ClosedLoop_Tracker::on_new_reading(goal);
        if (!mypub.publish(object)){
            throw "cannot publish!";
        }

    }
        


    ObjectPackage getObjectPackage(const Disturbance & goal){
        ObjectPackage object;
        if (window_area()>(ROBOT_HALFLENGTH*2)*(ROBOT_HALFWIDTH*2)){
            b2AABB attention_windowAABB;
            b2AABB attention_windowAABB.upperBound=b2Vec2(b2Transform_inf.p);
            b2AABB attention_windowAABB.lowerBound=b2Vec2(b2Transform_inf.p);
            attention_window.ComputeAABB(&attention_windowAABB, b2Transform_zero, 0);
            object.robot_high_x(attention_windowAABB.upperBound.x);
            object.robot_high_y(attention_windowAABB.upperBound.y);
            object.robot_low_x(attention_windowAABB.lowerBound.x);
            object.robot_low_y(attention_windowAABB.lowerBound.y);
        }
        //Di init (fake)
        if (Di.getAffIndex()!=NONE){
            object.Di_high_x(std::max_element(Di.vertices().begin(), Di.vertices().end(), CompareX));
            object.Di_high_y(std::max_element(Di.vertices().begin(), Di.vertices().end(), CompareY));
            object.Di_low_x(std::min_element(Di.vertices().begin(), Di.vertices().end(), CompareX));
            object.Di_low_y(std::min_element(Di.vertices().begin(), Di.vertices().end(), CompareY));            
        }
        if (goal.getAffIndex()!=NONE){
            object.goal_high_x(std::max_element(goal.vertices().begin(), goal.vertices().end(), CompareX));
            object.goal_high_y(std::max_element(goal.vertices().begin(), goal.vertices().end(), CompareY));
            object.goal_low_x(std::min_element(goal.vertices().begin(), goal.vertices().end(), CompareX));
            object.goal_low_y(std::min_element(goal.vertices().begin(), goal.vertices().end(), CompareY));            
        }
        return object;
    }
};


class UserInputConfigurator: public virtual Configurator{
    protected:
    DirectionSetter *directionSetter=NULL;
    AffordanceSetter *affordanceSetter=NULL;

    void explore_plan(b2World &world)override{
        if (worldBuilder.get_world_objects().size()==0){
            std::cout<<"ADD AN OBSTACLE PLEASE!"<<std::endl;
            return;
        }
        if (worldBuilder.get_world_objects().size()>1){
            throw "TOO MANY OBSTACLES!!";
        }
        std::cout<<iteration<<std::endl;
        if (iteration<=1){
            simResult result;            
            Disturbance disturbance;
            disturbance.bf=worldBuilder.get_world_objects()[0];
            disturbance.set_affordance(affordanceSetter->getAffIndex());
            disturbance.validate();
            vertexDescriptor v1=boost::add_vertex(transitionSystem);
            auto e=boost::add_edge(currentVertex, v1, transitionSystem);
            transitionSystem[v1].direction=directionSetter->getDirection();
            if(affordanceSetter->getAffIndex()==PURSUE && transitionSystem[v1].direction==DEFAULT){
                transitionSystem[v1].Dn=disturbance;
                transitionSystem[v1].Di=controlGoal.get_disturbance();
                transitionSystem[v1].endPose.p.x=disturbance.pose().p.x-0.07;
            }
            else{
                transitionSystem[v1].Di=disturbance;
                if (transitionSystem[v1].direction==DEFAULT){
                    float howFarShift=.5;
                    if (disturbance.pose().p.y<0) howFarShift=-howFarShift;
                    std::cout<<"howfar="<<howFarShift<<std::endl;
                    b2Transform newGoal;
                    newGoal.p=b2Vec2(0, howFarShift)+disturbance.pose().p;
                    debug::print_pose(newGoal, "newgoal");
                    controlGoal=Task(Disturbance(PURSUE, newGoal.p), UNDEFINED);
                    init(controlGoal);
                    register_tracker(tracker);
                }
            }
            currentTask.set_change(true);
            transitionSystem[e.first].step=20;
            transitionSystem[e.first].it_observed=iteration;
            debug::print_pose(transitionSystem[v1].Di.pose(), "Di:");
            debug::print_pose(transitionSystem[v1].Dn.pose(), "Dn:");
            m_plan={v1};
        }
        trackerInterface.getData(transitionSystem[v1].Di);
        
    }

    struct TrackerInterface{
        TrackerInterface()=default;
        QtTracker tracker;

        void getData(const Disturbance& Di){
            tracker.setDisturbance(Di);
        }
    }trackerInterface;
    public:
    UserInputConfigurator()=delete;

    UserInputConfigurator(DirectionSetter * ds, AffordanceSetter * as): Configurator(){
        directionSetter=ds;
        affordanceSetter=as;
    }

    ~UserInputConfigurator(){
        directionSetter=NULL;
        affordanceSetter=NULL;
    }
    public:
    void init(Task _task){
        Configurator::init(_task);
        register_tracker(&trackerInterface.tracker);
    }
};

class OneTaskController: public Wise_Controller{
    protected:
    public:
    OneTaskController()=default;

    void next_task(Task & currentTask, const Task & controlGoal, const TransitionSystem & g, std::vector <vertexDescriptor> & current_vertices, std::vector<vertexDescriptor> & plan)override{
        Wise_Controller::next_task(currentTask, controlGoal, g, current_vertices, plan);
        // if (currentTask.is_over() && currentTask.getAction().getLWheelSpeed()!=0 && currentTask.getAction().getRWheelSpeed()!=0){
        //     std::cout<<"terminating!"<<std::endl;
        //     currentTask=Task(Disturbance(), STOP);
        //     currentTask.getAction().setLWheelSpeed(0);
        //     currentTask.getAction().setRWheelSpeed(0);
        //     currentTask.set_change(false);
        // }
    }

};


// Disturbance set_target(int& run, b2Transform start){
// 	Disturbance result;
// 	return result;
// }
#undef DEBUG
#define DEBUG true

int main(int argc, char** argv) {
	A1Lidar lidar;
	AlphaBot motors;
	LIDAR_In configuratorInterface;
	Motor_Out controlInterface;
    AffordanceSetter as;
    DirectionSetter ds;
    std::cout<<as.getAffIndex()<<", "<<ds.getDirection()<<std::endl;
    UserInputConfigurator configurator(&ds, &as);
    b2Vec2 goalPos(1,0);
    Disturbance goal(PURSUE, goalPos);
    Task controlGoal(goal, UNDEFINED);
    configurator.init(controlGoal);
	OneTaskController rc;
	configurator.register_controller(&rc);
	if (argc>2){
		configuratorInterface.debugOn=atoi(argv[2]);
	}
	configurator.setSimulationStep(.5);
	//printf("current vertices size=%i\n", configurator.current_vertices.size());

	LidarInterface dataInterface(&configuratorInterface);
	configurator.registerInterface(&configuratorInterface, &controlInterface);
	MotorCallback cb(&controlInterface);
	lidar.registerInterface(&dataInterface);
	motors.registerStepCallback(&cb);
	configurator.start();
	lidar.start();
	motors.start();
	do{
    }while(!getchar());
	configurator.stop();
	motors.stop();
	lidar.stop();
}
	
	
