#include "ServoMotorSetting.h"

#include <cmath>
#include <cstdio>

/**
 * @brief default neutral high time in nanoseconds.
 */
ServoMotorSetting::ServoMotorSetting()
    : ServoMotorSetting(defaultNeutralHighTimeNs)
{
}

/**
 * @brief constructs a ServoMotorSetting object with a specified neutral high time.
 *
 * @param neutralHighTimeValueNs the neutral high time value in nanoseconds.
 */
ServoMotorSetting::ServoMotorSetting(
    const int neutralHighTimeValueNs)
    : neutralHighTimeNs(neutralHighTimeValueNs)
{
}

ServoMotorSetting::~ServoMotorSetting()
{
    stop();
}

int ServoMotorSetting::start(int channel, int chipNo)
{
    if (started)
        return setSpeed(0.0F);

    const int neutralPeriodNs =
        inverseDutyNs + neutralHighTimeNs;

    // start the PWM output directly at neutral speed.
    if (pwm.start(channel,
                  chipNo,
                  neutralPeriodNs,
                  inverseDutyNs) < 0) {
        return -1;
    }

    started = true;
    return 0;
}


int ServoMotorSetting::speedToHighTime(float speed) const// speed range from -1 to 1, 0 is stop
{
    // Limit commands to the supported normalized range.
    if (speed > 1.0F)
        speed = 1.0F;
    else if (speed < -1.0F)
        speed = -1.0F;

    // convert normalized speed to PWM high time in nanoseconds.
    return neutralHighTimeNs
          + static_cast<int>(speed * speedStepNs);

}

int ServoMotorSetting::setSpeed(const float speed)
{
    if (!started || !std::isfinite(speed))
        return -1;

    const int periodNs =
        inverseDutyNs + speedToHighTime(speed);

    int result = pwm.setPeriod(periodNs);

    if (result < 0) {
        std::fprintf(stderr,
                     "Cannot set motor PWM period.\n");
        return result;
    }

    result = pwm.setDutyCycleNS(inverseDutyNs);

    if (result < 0) {
        std::fprintf(stderr,
                     "Cannot set motor PWM duty cycle.\n");
    }

    return result;
}

int ServoMotorSetting::stop()
{
    if (!started)
        return 0;

    // request neutral speed before disabling PWM.
    const int speedResult = setSpeed(0.0F);

    if (speedResult < 0) {
        std::fprintf(stderr,
                     "Could not set motor speed to neutral.\n");
    }

    const int pwmResult = pwm.disable();

    if (pwmResult < 0) {
        std::fprintf(stderr,
                     "Could not disable the motor PWM.\n");
    } else {
        started = false;
    }

    return speedResult < 0 ? speedResult : pwmResult;
}
