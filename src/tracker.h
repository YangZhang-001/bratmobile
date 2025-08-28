#include "sensor.h"

/**
 * @brief Tracking interface: Bridge between the real world and the simulation. Is used for tracking execution of tasks
 * and using lived experience to modify the state-matching threshold
 * 
 */
class Tracker{
    protected:
    friend Configurator;
    ThresholdLearner *learner=NULL;
    b2Transform deltaTransform=b2Transform_zero;
    public:
    Threshold threshold=Threshold();

    Tracker(){}

    Threshold * get_threshold(){
        return &threshold;
    }

    void register_learner(ThresholdLearner * l){
        learner=l;
    }

    b2Transform getDeltaTransform(){return deltaTransform;}
    /**
    * calculates 2d affine transformation of input task's disturbance from t-1 to t. In other words, expresses how much the disturbance has moved
    * @param t input task
    * @param pts point cloud
    * @param objects world objects as extracted in worldbuilder
    */
    virtual b2Transform get_transform(const Task &t, const CoordinateContainer &pts, Disturbance * observed_disturbance, std::vector <BodyFeatures> & objects)=0; 

    /**
     * @brief Tracks task execution
     * 
    * @param t input task
    * @param pts point cloud
    * @param observed_disturbance pointer body features of the observed disturbance. 
    * @param objects world objects as extracted in worldbuilder
     * @return b2Transform that the robot has moved by, can use for updating cognitive map and control
     */
    virtual b2Transform track(Task &t, const CoordinateContainer &pts, std::vector <BodyFeatures> & objects)=0;
    //void adjust_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);                

    /**
     * @brief Called every time a new task is created
     * 
     * @param t the task
     */
    virtual void on_new_task(const Task &task)=0;

    /**
     * @brief Called every time asensor reading is available
     * 
     * @param task 
     */
    virtual void on_new_reading(Task * task=NULL)=0;

    virtual void init(Task * goal)=0;


    /**
    * @brief opens file where all the data is dumped
    */
    void make_log();
    
    protected:


    void log_thresholds(){
        FILE *f= fopen("/tmp/thresholds.txt", "a");
        fprintf(f, "%f\t%f\t%f\t%f\t%f\n", threshold.for_Di().get_x(),
                                            threshold.for_Di().get_y(),
                                            threshold.for_Di().get_angle(),
                                            threshold.for_Di().get_width(),
                                            threshold.for_Di().get_length());
        fclose(f);
    }

    
};

/**
 * @brief Tracks task execution through dead reckoning
 * 
 */
class DeadReckoner: public Tracker{
    public:
    DeadReckoner(){}

    b2Transform get_transform(const Task &t, const CoordinateContainer &pts, Disturbance * observed_disturbance, std::vector <BodyFeatures> & objects){
        return t.getAction().getTransform(LIDAR_SAMPLING_RATE);
    }     
    
    b2Transform track(Task &t, const CoordinateContainer &pts, std::vector <BodyFeatures> & objects);

    void on_new_task(const Task &task){} //does nothing

    void on_new_reading(Task * task=NULL){};

    void init(Task * goal){}


};

/**
 * @brief Tracks task execution by observing changes in tracked disturbance
 * 
 */
class ClosedLoop_Tracker:public Tracker{
    protected:
    Disturbance tracked_disturbance; //disturbance to be tracked as at task start, kept in memory when task is changed
    b2PolygonShape attention_window; //a box drawn at the beginning of task which bounds the robot and the goal
    
     /**
    * @brief gets the area of the attention window (for debugging)
    */
    float window_area();

    public:

    ClosedLoop_Tracker(){}

    // ClosedLoop_Tracker(Task * goal){
	//     attention_window=sensor_box(Robot::get_vertices(),b2Transform_zero, goal->get_disturbance());
    // }

    /**
    * @brief returns 2d transformation matrix between one scan and the next based on the displacement of disturbance Di for a task
    *
    * @param t the current task
    * @param pts lidar reading
    * @param observed_disturbance disturbance Di for task t
    * @param objects objects in the world (stored in worldbuilder)
    */
    b2Transform get_transform(const Task &t, const CoordinateContainer &pts, Disturbance * observed_disturbance, std::vector <BodyFeatures> & objects);    
    
    b2Transform track(Task &t, const CoordinateContainer &pts, std::vector <BodyFeatures> & objects);

    /**
    * @brief returns an upright rectangle which represents a focus of attention for finding points corresponding to input task's disturbance
    */
    cv::Rect2f real_world_focus(const Task * );

    Disturbance * get_tracked_disturbance(){
        return &tracked_disturbance;
    }

    void set_tracked_disturbance(const Disturbance & d){
        tracked_disturbance=d;
    }

    /**
    * @brief Get disturbance to be tracked among the worldbuilder objects
    * 
    * @param objects worldBuilder objects
    * @param dist disturbance to be tracked
    * @param t the estimated instantaneous 2d transform associated to the currently executed task
    */
    std::vector <BodyFeatures>::iterator find_disturbance(std::vector <BodyFeatures> & objects, const BodyFeatures & dist, b2Transform t, float * _least_square=NULL);

    /**
     * @brief Uses the goal to reset tracked disturbance at each task
     * 
     * @param task the new task
     */
    void on_new_task(const Task& task);

    /**
     * @brief Updates the attention window at each sensor reading
     * 
     * @param goal the goal
     */
    virtual void on_new_reading(const Task & goal, const Task &currentTask);

    void set_attention(b2PolygonShape ps){
        attention_window=ps;
    }

    void init(Task * goal){
        attention_window=sensor_box(Robot::get_vertices(),b2Transform_zero, goal->get_disturbance_ptr());

    }




};