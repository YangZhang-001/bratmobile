#include "tracker.h"

TrackingResult Tracker::get_transform(const Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects){
    TrackingResult result;
    result.displacement= t.getAction().getTransform(LIDAR_SAMPLING_RATE);
    result.observed_disturbance=t.get_disturbance();
    result.observed_disturbance.setPose(b2Mul(-result.displacement, t.get_disturbance().pose()));
    return result;
}

bool Tracker::hasTaskEnded(Task & t){
    return t.getMotorStep()<1;
}

bool ClosedLoop_Tracker::hasTaskEnded(Task & t){
    bool ended=t.checkEnded(attention_window, b2Transform_zero, &tracked_disturbance); //the attention_window moves with the robot
    return t.getMotorStep()==0 || ended;
}

TrackingResult DeadReckoner::track(Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects){
    TrackingResult result=get_transform(t, pts, objects);
    //math::MulT(-deltaTransform, *t.get_disturbance_ptr());
    t.setMotorStep(t.getMotorStep()-1); //this should go in the controller
    // if (t.getMotorStep()<1){
    //     t.set_change(true);
    // }
    deltaTransform=b2Mul(result.displacement, deltaTransform);
    return result;
}

TrackingResult ClosedLoop_Tracker::track(Task &t, const CoordinateContainer &pts, const std::vector <BodyFeatures> & objects){
    TrackingResult result=get_transform(t, pts, objects);
	// bool ended=t.checkEnded(attention_window, b2Transform_zero, &tracked_disturbance); //the attention_window moves with the robot
	// if(t.getMotorStep()==0 || ended){
	// 	t.set_change(true);
	// }    
    deltaTransform=b2Mul(result.displacement, deltaTransform);
    return result;
}


cv::Rect2f ClosedLoop_Tracker::real_world_focus(const Task * t){
    std::vector <cv::Point2f> vertices;
    if (t->get_disturbance().getAffIndex()==NONE){
        return cv::Rect2f(0, 0, 0, 0);
    }
    cv::Point2f bl; //bottom left (documentation CV says top left but not true)
    float max_dimension=std::max(t->get_disturbance().bf.width(), t->get_disturbance().bf.length());
    max_dimension+=0.02;    
    bl.x=t->get_disturbance().pose().p.x-(max_dimension/2);
    bl.y=t->get_disturbance().pose().p.y-(max_dimension/2);
    cv::Rect2f focus(bl.x, bl.y, max_dimension, max_dimension);
    return focus; //upright bounding rectangle: increases area represented
}


TrackingResult ClosedLoop_Tracker::get_transform(const Task & t, const CoordinateContainer & pts, const std::vector <BodyFeatures> & objects){
    TrackingResult result=Tracker::get_transform(t, pts, objects);
    if (t.get_disturbance().getAffIndex()==NONE || t.get_disturbance().bf.is_point()|| (t.getAction().getLWheelSpeed()==0 && t.getAction().getRWheelSpeed()==0)){
        if (t.get_disturbance().getAffIndex()==NONE){
            std::cerr<<"no disturbance!"<<std::endl;    
        }
        if (t.get_disturbance().bf.is_point()){
            printf("petite disturbance!");    
        }
        if ((t.getAction().getLWheelSpeed()==0 && t.getAction().getRWheelSpeed()==0)){
            std::cerr<<("not moving!")<<std::endl;    
        }
        //return t.getAction().getTransform(LIDAR_SAMPLING_RATE);
        return result;
    }
    // BodyFeatures predicted_bf=t.get_disturbance().bf;
    // predicted_bf.pose=b2Mul(t.getAction().getTransform(LIDAR_SAMPLING_RATE), predicted_bf.pose);
    auto new_d_it =find_disturbance(objects.cbegin(), objects.cend(), result.observed_disturbance.bodyFeatures(), t.getAction().getTransform(LIDAR_SAMPLING_RATE));
    if (new_d_it==objects.end()){
        printf("not found!");
        result.observed_disturbance.set_affordance(NONE); //this will tell the task that D is null, so it can end!
        return result;
    }
    if ((*new_d_it).is_point()){
        throw std::invalid_argument("for some reason it's tiny!");    
    }
    //BodyFeatures new_d=*new_d_it;
   // result.displacement=b2Transform_zero;
    result.observed_disturbance=*new_d_it;
    correctAngle(result.observed_disturbance.bf, t.get_disturbance().bodyFeatures());
    calc_transform(result.displacement, result.observed_disturbance.pose(), t.get_disturbance().pose());
    //result.observed_disturbance.bf=new_d; //this modifies task t, do not move!
    result.displacement=-result.displacement;
    return result;
}

void Tracker::make_log(){
    if (learner){
        learner->make_log();
    }
    FILE * f=fopen("/tmp/thresholds.txt", "w");
    fclose(f);
}

std::vector <BodyFeatures>::const_iterator ClosedLoop_Tracker::find_disturbance( std::vector <BodyFeatures>::const_iterator objects_begin, std::vector <BodyFeatures>::const_iterator objects_end, const BodyFeatures & dist, b2Transform t, float * _least_square){
    float least_square=10000;
   std::vector <BodyFeatures>::const_iterator result =objects_end;
    //for (std::vector <BodyFeatures>::iterator it=objects_begin; it!=objects.end(); it++){
    while (objects_begin!=objects_end){
    Bundle distance;
    bool match =(*objects_begin).match(dist, &distance, t);
    if (float ss=distance.sum_squares()<least_square){
        least_square=ss;
        if (match){ //thresholding
            result = objects_begin;
        }
        else if(Disturbance d(*objects_begin); overlaps(attention_window, &d)){
            result=objects_begin;
            //adjust threshold
            Bundle error=threshold.for_Di()-distance;
            if (learner){
                printf("but it's still there!");
                learner->Di_tune(error, threshold.for_Di());
                threshold.set_Di(learner->update_bundle(error, threshold.for_Di()));    
            }
            printf("overlaps but theres error! DISTANCE! x=%f \ty%f\ttheta=%f\tw=%f\tl%f\t", distance.get_x(), distance.get_y(), distance.get_angle(),distance.get_width(), distance.get_length());
        }
    }
    objects_begin++;
    }  
    log_thresholds();
    return result;
}

void ClosedLoop_Tracker::correctAngle(BodyFeatures & found, const BodyFeatures & dist){
    if (fabs(found.pose.q.GetAngle()-dist.pose.q.GetAngle())>(3*M_PI_4)){
        if (dist.pose.q.GetAngle()>0){
            found.pose.q.Set(found.pose.q.GetAngle()-M_PI);
        }
        else if (dist.pose.q.GetAngle()<0){
            found.pose.q.Set(found.pose.q.GetAngle()+M_PI);
        }
    }

        //     if (dist.pose.q.GetAngle()>0){
        //     found.pose.q= b2Mul(b2Rot(-M_PI),found.pose.q);
        // }
        // else if (dist.pose.q.GetAngle()<0){
        //     found.pose.q =b2Mul(b2Rot(M_PI),found.pose.q);
        // }


}

void ClosedLoop_Tracker::on_new_task(const Task & task){
    tracked_disturbance=task.get_disturbance();
    deltaTransform=b2Transform_zero;
}

void ClosedLoop_Tracker::on_new_reading(const Task & goal, const Task & currentTask){
    printf("new reading!\n");
    float area=0;
    // if(goal){
    //     std::cout<<"no goal!"<<std::endl;
    //     return;
    // }
    Disturbance goalD=goal.get_disturbance();
    if (currentTask.get_disturbance().getAffIndex()==PURSUE && goal.get_disturbance().getAffIndex()){
        goalD=Disturbance();
    }
    try{
        attention_window=sensor_box(Robot::get_vertices(),b2Transform_zero, goalD);
        if (area=window_area(); area<(ROBOT_HALFLENGTH*2)*(ROBOT_HALFWIDTH*2)){
            throw area;
        }
    }
    catch (float the_area){
        std::cerr<< "no attention! area: "<<the_area<<std::endl;
    }
}

float ClosedLoop_Tracker::window_area(){
    b2AABB aabb;
    attention_window.ComputeAABB(&aabb, b2Transform_zero, 0);
    float base= fabs(aabb.upperBound.x-aabb.lowerBound.x);
    float height =fabs(aabb.upperBound.y-aabb.lowerBound.y);
    return base*height;
}

