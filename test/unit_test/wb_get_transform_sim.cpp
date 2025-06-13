#include "../callbacks.h"

int main(int argc, char ** argv){
    Disturbance target= Disturbance(PURSUE, b2Vec2(1.0,0), 0);    
    Task goal(target,DEFAULT);
    LIDAR_In ci;
    WorldBuilder wb;
    DataInterface di(&ci);  
    if (argc>1){
        di.folder=argv[1];
    }
    Task task;
    ClosedLoop_Tracker tracker;
    tracker.init(&task); //makes sensor box
    Disturbance d;
    b2Transform target_shift=b2Transform_zero;
    for (int i=0; i<10; i++){
        di.newScanAvail();          
        b2World w(b2Vec2_zero);
        wb.world_objects=wb.getFeatures(ci.data2fp, b2Transform_zero);
        wb.buildWorld(w, b2Transform_zero, DEFAULT, task.disturbance, 0.15, WorldBuilder::PARTITION );
        Robot robot(&w);
        simResult result =task.bumping_that(w, 0, robot.body);
        b2Transform delta_d=b2Transform_zero; 
        if (i==0){
            d=result.collision;
            d.set_affordance(PURSUE);
            task=Task(d, DEFAULT, b2Transform_zero, true);
            tracker.set_attention(sensor_box(Robot::get_vertices(),b2Transform_zero, &(task.disturbance)));
        } 
        else{
            delta_d=task.disturbance.pose()-result.collision.pose(); 
        }
        Disturbance d;
        b2Transform deltaPose=tracker.get_transform(task, ci.data2fp, &d, wb.world_objects); //track using obstacle OR dead reckoning
        target_shift+=deltaPose;
        if (deltaPose.p!=delta_d.p){
            b2Transform error=delta_d-deltaPose;
            printf("X error=%f y error=%f theta error=%f\n",error.p.x ,error.p.y, error.q.GetAngle());
           // if (error.p.x>delta_d.p.x || error.p.y>delta_d.p.y || error.q.GetAngle()>delta_d.q.GetAngle()  ){
              //throw "way off";
            // }
        }
        d.set_affordance(PURSUE);
        task.disturbance=d;
    
    }
    return 0;

}