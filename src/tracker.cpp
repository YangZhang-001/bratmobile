#include "tracker.h"

b2Transform DeadReckoner::track(Task &t, const CoordinateContainer &pts, std::vector <BodyFeatures> & objects){
    b2Transform result=get_transform(t, pts, t.get_disturbance(), objects);
    math::applyAffineTrans(-result, t.disturbance);
    t.motorStep--;
    if (t.motorStep<1){
        t.change=true;
    }
    return result;
}

b2Transform ClosedLoop_Tracker::track(Task &t, const CoordinateContainer &pts, std::vector <BodyFeatures> & objects){
    b2Transform result=get_transform(t, pts, t.get_disturbance(), objects);
	bool ended=t.checkEnded(attention_window, b2Transform_zero, &tracked_disturbance); //the attention_window moves with the robot
	if(t.motorStep==0 || ended){
		t.change=1;
	}    
    return result;
}


cv::Rect2f ClosedLoop_Tracker::real_world_focus(const Task * t){
    std::vector <cv::Point2f> vertices;
    if (t->disturbance.getAffIndex()==NONE){
        return cv::Rect2f(0, 0, 0, 0);
    }
    cv::Point2f bl; //bottom left (documentation CV says top left but not true)
    float max_dimension=std::max(t->disturbance.bf.width(), t->disturbance.bf.length());
    max_dimension+=0.02;    
    bl.x=t->disturbance.pose().p.x-(max_dimension/2);
    bl.y=t->disturbance.pose().p.y-(max_dimension/2);
    cv::Rect2f focus(bl.x, bl.y, max_dimension, max_dimension);
    return focus; //upright bounding rectangle: increases area represented
}


b2Transform ClosedLoop_Tracker::get_transform(const Task & t, const CoordinateContainer & pts, Disturbance * observed_disturbance, std::vector <BodyFeatures> & objects){
    if (observed_disturbance==NULL){
        throw std::invalid_argument("disturbance pointer cannot be null!");
    }
    if (t.disturbance.getAffIndex()==NONE || t.disturbance.bf.is_point()|| (t.action.getLWheelSpeed()==0 && t.action.getRWheelSpeed()==0)){
        if (t.disturbance.getAffIndex()==NONE){
            throw std::invalid_argument("no disturbance!");    
        }
        if (t.disturbance.bf.is_point()){
            printf("petite disturbance!");    
        }
        if ((t.action.getLWheelSpeed()==0 && t.action.getRWheelSpeed()==0)){
            throw std::invalid_argument("not moving!");    
        }
        return t.action.getTransform(LIDAR_SAMPLING_RATE);
    }
    BodyFeatures predicted_bf=t.disturbance.bf;
    predicted_bf.pose+=t.action.getTransform(LIDAR_SAMPLING_RATE); //future to sub with MM Kalman
    auto new_d_it =find_disturbance(objects, predicted_bf, t.action.getTransform(LIDAR_SAMPLING_RATE));
  //  printf("objects: %i\n", objects.size());
    if (new_d_it==objects.end()){
        printf("not found!");
        observed_disturbance->set_affordance(NONE); //this will tell the task that D is null, so it can end!
        return t.action.getTransform(LIDAR_SAMPLING_RATE);
    }
    if ((*new_d_it).is_point()){
        throw std::invalid_argument("for some reason it's tiny!");    
    }
    BodyFeatures new_d=*new_d_it;
    b2Transform result=b2Transform_zero;
    calc_transform(result, new_d.pose, t.disturbance.pose());
    observed_disturbance->bf=new_d; //this modifies task t, do not move!
    return -result;
}

std::vector <BodyFeatures>::iterator ClosedLoop_Tracker::find_disturbance( std::vector <BodyFeatures> & objects, const BodyFeatures & dist, b2Transform t, float * _least_square){
    float least_square=10000;
    std::vector <BodyFeatures>::iterator result =objects.end();
    for (std::vector <BodyFeatures>::iterator it=objects.begin(); it!=objects.end(); it++){
        Bundle distance;
        bool match =(*it).match(dist, &distance, t);
        if (float ss=distance.sum_squares()<least_square){
            least_square=ss;
            if (match){ //thresholding
                result = it;
            }
            else if(Disturbance d(*it); overlaps(attention_window, &d)){
                result=it;
                //adjust threshold
                Bundle error=threshold.for_Di()-distance;
                if (learner){
                    printf("but it's still there!");
                    learner->Di_tune(error, threshold.for_Di());
                    threshold.set_Di(learner->update_bundle(error, threshold.for_Di()));    
                }
                printf("DISTANCE! x=%f \ty%f\ttheta=%f\tw=%f\tl%f\t", distance.get_x(), distance.get_y(), distance.get_angle(),distance.get_width(), distance.get_length());
            }
        }
    }
    if (result!=objects.end()){
        if (fabs((*result).pose.q.GetAngle()-dist.pose.q.GetAngle())>(3*M_PI_4)){
            if (dist.pose.q.GetAngle()>0){
                (*result).pose.q.Set((*result).pose.q.GetAngle()-M_PI);
            }
            else if (dist.pose.q.GetAngle()<0){
                (*result).pose.q.Set((*result).pose.q.GetAngle()+M_PI);
            }
        }
    }
    log_thresholds();
    return result;
}

void ClosedLoop_Tracker::on_new_task(Task *task){
    if (!task){
        throw "no task!";
    }
    tracked_disturbance=*task->get_disturbance();
}

void ClosedLoop_Tracker::on_new_reading(Task * goal){
	attention_window=sensor_box(Robot::get_vertices(),b2Transform_zero, goal->get_disturbance());
}

void ClosedLoop_Tracker::make_log(){
    if (learner){
        learner->make_log();
    }
    FILE * f=fopen("/tmp/thresholds.txt", "w");
    fclose(f);
}
