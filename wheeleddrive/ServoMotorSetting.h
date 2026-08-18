#ifndef SERVO_MOTOR_SETTING_H
#define SERVO_MOTOR_SETTING_H
//#include <iostream>
#include "Pwm.h"

class ServoMotorSetting
{
public:

    ServoMotorSetting();
    /**
     * @brief Constructs a ServoMotorSetting object with a specified neutral high time.
     *
     * @param neutralHighTimeValueNs The neutral high time value in nanoseconds.
     */
    explicit ServoMotorSetting(int neutralHighTimeValueNs);
    ~ServoMotorSetting();

    // One object owns one PWM motor output.
    ServoMotorSetting(const ServoMotorSetting &) = delete;
    ServoMotorSetting &operator=(const ServoMotorSetting &) = delete;

    /**
     * @brief Starts one motor at neutral speed.
     *
     * @param channel PWM channel number.
     * @param chipNo PWM chip number.
     * @return 0 on success, otherwise -1.
     */
    int start(int channel, int chipNo);

    /**
     * @brief Sets a normalized motor speed from -1 to +1.
     */
    int setSpeed(float speed);

    /**
     * @brief Sets neutral speed and disables PWM.
     */
    int stop();

private:
    /**
     * @brief Converts normalized speed to PWM high time.
     */
    int speedToHighTime(float speed) const;

    PWM pwm;
    bool started = false;
    const int neutralHighTimeNs;

    static constexpr int inverseDutyNs = 20000000;
    static constexpr int defaultNeutralHighTimeNs = 1524000;
    static constexpr int speedStepNs = 100000;
};

#endif
