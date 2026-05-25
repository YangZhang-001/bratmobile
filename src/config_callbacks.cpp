#include "config_callbacks.h"

void MotorInterface::adjust_gain (float angle_D, b2Transform observed,
                                  float *y_D)
{
    float angle_error = angle_D - observed.q.GetAngle ();
    float y_error = 0;
    if (y_D != NULL)
    {
        printf ("outer loop\n");
        if (fabs (observed.p.y) < (*y_D))
        {
            //y only
            y_error = (*y_D) - observed.p.y;
            angle_error = outer_loop (y_error);
        }
    }
    PID (angle_error);
}

void MotorInterface::PID (float e)
{
    integral += e;
    float increment = Kp * e + Ki * integral + Kd * (prev_error - e);
    L_gain -= increment / 2;
    R_gain += increment / 2;
    prev_error = e;
}

float MotorInterface::outer_loop (float e)
{
    return 0.1
           * e; //0.1 is Kp for outer loop but don't know why it doesnt let me set it inside MotorInterface
}
