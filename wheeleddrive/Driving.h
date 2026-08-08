#ifndef DRIVING_H
#define DRIVING_H

#include "ServoMotorSetting.h"

#include <mutex>

class Driving
{
public:

    Driving();
    ~Driving();

    // one driving object owns both motor outputs.
    Driving(const Driving &) = delete;
    Driving &operator=(const Driving &) = delete;

    /**
     * @brief Starts both motors at neutral speed
     * Throws an exeption if not successful.
     */
    void start();

    /**
     * @brief Sets the speeds of the motors
     * This sets the speeds in a way that a positive speed = forward
     * and a neg speed is backwards.
     *
     * @param left_speed A value between -1 and +1
     * @param right_speed A speed value between -1 and +1
     */
    int setMotorSpeeds(float left_speed, float right_speed);

    /**
     * @brief Stop the robot and switch off PWM.
     **/
    void stop();

private:

    // Measured neutral pulse for each physical wheel servo.
    static constexpr int leftNeutralHighTimeNs = 1521500;
    static constexpr int rightNeutralHighTimeNs = 1524000;

    // Servo motor settings for left and right motors
    ServoMotorSetting leftMotor;
    ServoMotorSetting rightMotor;

    // protect motor commands against concurrent shutdown.
    std::mutex motorMutex;
    bool started = false;

    // Constants for servo motor settings
    static constexpr int leftChannel = 0;
    static constexpr int rightChannel = 0;
    static constexpr int leftChipNo = 2;
    static constexpr int rightChipNo = 1;
};

#endif
