#include "test_classes.h"
#include <gtest/gtest.h>


class MotorTimer: public CppTimer{
    Motor_Out * mo=NULL;
    public:
    MotorTimer(Motor_Out * _mo): mo(_mo){}

    void timerEvent(){
        mo->decrease_motorStep();
        if (mo->get_motorStep()==0){
            stop();
        }
    }
};


TEST(DeadReckoning, test){
    Task::Action action;
    action.set_motorStep(20);
    Motor_Out motor;
    motor.getData(action);
    MotorTimer timer(&motor);
    timer.startms(100);
    timer.stop();

}