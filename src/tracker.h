#ifndef TRACKER_H
#define TRACKER_H
#include "sensor.h"

struct TrackingResult{
    b2Transform displacement=b2Transform_zero; //estimated displacement
    Disturbance observed_disturbance; //the disturbance as observed at the current time step

    TrackingResult()=default;

    TrackingResult(const Disturbance& d, const b2Transform & tr=b2Transform_zero): displacement(tr), observed_disturbance(d){}
};

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
    Threshold threshold=Threshold();

    public:

    Tracker(){}

    virtual Threshold get_threshold(const State &s){
        return threshold;
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
    virtual TrackingResult get_transform(const Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects);
    /**
     * @brief Tracks task execution
     * 
    * @param t input task
    * @param pts point cloud
    * @param observed_disturbance pointer body features of the observed disturbance. 
    * @param objects world objects as extracted in worldbuilder
     * @return b2Transform that the robot has moved by, can use for updating cognitive map and control
     */
    virtual TrackingResult track(const Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects)=0;
    //void adjust_task(const vertexDescriptor&, TransitionSystem &, Task*, const b2Transform &);                

    /**
     * @brief Called every time a new task is created
     * 
     * @param t the task
     */
    virtual void on_new_task(const Task &task, const Task & goal)=0;

    /**
     * @brief Called every time a new reading is available
     * 
     * @param goal the goal task
     * @param task current Task
     */
    virtual void on_new_reading(const Task & goal,const Task &task ){}

    virtual void init(const Task & goal)=0;

    virtual bool hasTaskEnded(Task & t);


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
    
    TrackingResult track(const Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects);

    void on_new_task(const Task &task, const Task & goal)override{} //does nothing

    void init(const Task & goal){}


};

/**
 * @brief Tracks task execution by observing changes in tracked disturbance
 * 
 */
class ClosedLoop_Tracker:public Tracker{
    protected:
    Disturbance tracked_disturbance; //disturbance to be tracked as at task start, kept in memory when task is changed
    b2PolygonShape attention_window; //a box drawn at the beginning of task which bounds the robot and the goal
    bool hasReading=false; //has the disturbance's actual position been found with a scan
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
    * @brief returns 2d transformation matrix between one scan and the next based on the displacement of disturbance Di for a task +
    * the disturbance as observed (this is to update the noisy measurement)
    *
    * @param t the current task
    * @param pts lidar reading
    * @param observed_disturbance disturbance Di for task t
    * @param objects objects in the world (stored in worldbuilder)
    */
    TrackingResult get_transform(const Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects)override;    
    
    TrackingResult track(const Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects);

    /**
    * @brief returns an upright rectangle which represents a focus of attention for finding points corresponding to input task's disturbance
    */
    cv::Rect2f real_world_focus(const Task * );

    /**
    * @brief Get disturbance to be tracked among the worldbuilder objects
    * 
    * @param objects worldBuilder objects
    * @param dist disturbance to be tracked
    * @param t the estimated instantaneous 2d transform associated to the currently executed task
    */
    virtual std::vector <BodyFeatures>::const_iterator find_disturbance(std::vector <BodyFeatures>::const_iterator objects_begin, std::vector <BodyFeatures>::const_iterator objects_end, const BodyFeatures & dist, b2Transform t, float * _least_square=NULL);

    /**
     * @brief Sets angle to be smallest possible increment compared to dist
     * @param found the found disturbance
     * @param dist the disturbance to be tracked
     */
    void correctAngle(BodyFeatures & found, const BodyFeatures & dist);
    /**
     * @brief Uses the goal to reset tracked disturbance at each task
     * 
     * @param task the new task
     * @param goal the goal
     */
    void on_new_task(const Task &task, const Task & goal)override;


    void set_attention(b2PolygonShape ps){
        attention_window=ps;
    }

    void init(const Task & goal){
        attention_window=sensor_box(Robot::get_vertices(),b2Transform_zero, goal.get_disturbance());

    }

    virtual bool hasTaskEnded(Task & t);

    /**
     * @brief Updates the attention window at each sensor reading
     * 
     * @param goal the goal
     */
    void makeAttentionWindow(const Task &goal, const Task & currentTask);
    
};



//same as CL Tracker but creates custom threshold based on state
class CLAdaptiveTracker:public ClosedLoop_Tracker{
    protected:

    Threshold get_threshold(const State &s){
        float distance=std::max(Threshold::FIXED_ENDPOSE, s.distance()/2);
//        float dist=std::max(Threshold::FIXED_DISTPOS, s.distance()/2);
        float dimensions=std::max(Threshold::FIXED_DIMENSIONS, logistic(getBiggestDimension(s)));
        return Threshold(distance, Threshold::FIXED_ANGLE, dimensions, Threshold::FIXED_AFFORDANCE, dimensions);
    }

    float getBiggestDimension(const State &s){
        return std::max(getMaxDim(s.Di), getMaxDim(s.Dn));
    }

    float getMaxDim(const Disturbance &d){
        return std::max(d.bf.halfLength, d.bf.halfWidth); 
    }

    /**
    @brief find a factor to multiply biggest disturbance dimension by in order to find match 
    */
    float logistic(float biggest){
        return (1/tanh(biggest))*0.2*biggest;
    }


};
#endif