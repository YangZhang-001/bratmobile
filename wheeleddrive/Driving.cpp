#include "Driving.h"

#include <stdexcept>
#include <cstdio>
#include <cmath>

// constructor
Driving::Driving()
    : leftMotor(leftNeutralHighTimeNs),
      rightMotor(rightNeutralHighTimeNs)
{
}

Driving::~Driving()
{
    stop();
}

void Driving::start()
{
    std::lock_guard<std::mutex> guard(motorMutex);

    if (started)
        return;

    if (leftMotor.start(leftChannel, leftChipNo) < 0) {
        throw std::runtime_error(
            "Driving: could not start the left motor");
    }

    if (rightMotor.start(rightChannel, rightChipNo) < 0) {
        // Stop the left motor when right motor startup fails.
        leftMotor.stop();

        throw std::runtime_error(
            "Driving: could not start the right motor");
    }

    // Both motor outputs were configured at neutral speed.
    started = true;
}

void Driving::stop()
{
    std::lock_guard<std::mutex> guard(motorMutex);

    // always try to stop both motors.
    const int leftResult = leftMotor.stop();
    const int rightResult = rightMotor.stop();

    started = false;

    if (leftResult < 0 || rightResult < 0) {
        std::fprintf(stderr,
                     "Could not stop all wheel motors cleanly.\n");
    }
}

int Driving::setMotorSpeeds(const float left_speed, const float right_speed)
{
    std::lock_guard<std::mutex> guard(motorMutex);

    if (!started) {
        std::fprintf(stderr,
                     "Driving has not been started.\n");
        return -1;
    }

    if (!std::isfinite(left_speed)
        || !std::isfinite(right_speed)) {
        std::fprintf(stderr,
                     "Motor speeds must be finite.\n");
        return -1;
    }

    // apply the calibrated left-wheel response correction
    float adjustedLeftSpeed = left_speed;

    if (left_speed > 0.0F)
        adjustedLeftSpeed *= leftForwardSpeedScale;
    else if (left_speed < 0.0F)
        adjustedLeftSpeed *= leftReverseSpeedScale;

    int result = leftMotor.setSpeed(adjustedLeftSpeed);

    if (result < 0) {
        std::fprintf(stderr,
                     "Could not set left motor speed.\n");

        // return both motors to neutral after a command failure.
        leftMotor.setSpeed(0.0F);
        rightMotor.setSpeed(0.0F);

        return result;
    }

    // the right motor is mounted in the opposite direction.
    result = rightMotor.setSpeed(-right_speed);

    if (result < 0) {
        std::fprintf(stderr,
                     "Could not set right motor speed.\n");

        leftMotor.setSpeed(0.0F);
        rightMotor.setSpeed(0.0F);

        return result;
    }

    return 0;
}
