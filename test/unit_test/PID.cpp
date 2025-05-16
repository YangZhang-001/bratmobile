#include "control_interface.h"

/**
 * @brief Uses kinematic model to calculate angle
 * @param l left wheel speed (normalised)
 * @param r right wheen speed (normalised)
 */
float kinematic_angle(float l, float r, float v){
    return (v*(r-l)/BETWEEN_WHEELS);
}

int main(int argc, char** argv){
    float Kp=0.02, Ki=0.008, Kd=0.003;
    if (argc>3){
        Kp=atof(argv[1]);
        Ki=atof(argv[2]);
        Kd=atof(argv[3]);
    }
    Motor_Out mo(Kp, Ki, Kd);
    Task::Action action;
    action.setLWheelSpeed(0.5);
    action.setRWheelSpeed(0.5);
    mo.getData(action);
    //float max_error_dt=kinematic_angle(1.0, -1.0, MAX_SPEED)*LIDAR_SAMPLING_RATE;
    float max_error_dt=0.1;
    float error_tolerance=0.025;
    srand(time(NULL));
    int _roof=(fabs(max_error_dt)*1000);
    float pos=(rand()%100)%2;
    //test instantaneous angle correction
    int count=0;
    for (int i=-100; i<100; i+=2){
        // int _start=( rand()%_roof);
        float start_float=float(i)/1000;
        if (pos==0){
            start_float=-start_float;
        }
        mo.PID(start_float);
        float delta_angle=kinematic_angle(mo.get_L(), mo.get_R(), MAX_SPEED);
        if(fabs(start_float+delta_angle)>error_tolerance){
            printf("error %f with start angle=%f\n", start_float+delta_angle, start_float);
            count++;
        }
        std::cerr <<"iteration "<< i<<": "<<delta_angle<<std::endl;
        mo.reset();
    }
    std::cout<<count;
    if (count>75){
        return 1;
    }
    return 0;
}