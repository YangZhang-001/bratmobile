#ifndef PWM_H
#define PWM_H

//#include <iostream>
//#include <math.h>
#include <string>
//#include <stdlib.h>
//#include <unistd.h>

//using namespace std;

/**
 * @brief control one PWM channel through Linux sysfs interface on rock5b
 */
class PWM
{
public:

    PWM() = default;
    ~PWM();

    // one object owns one hardware PWM channel.
    PWM(const PWM &) = delete;
    PWM &operator=(const PWM &) = delete;

    /**
     * @brief Start the PWM output
     *
     * @param channel PWM channel (e.g., 0 for pwm0)
     * @param chip PWM chip number
     * @param periodNs Initial PWM period in nanoseconds
     * @param dutyCycleNs Initial duty cycle in nanoseconds.
     * @return 0 success, -1 failed
     */
    int start(int channel, int chip, int periodNs, int dutyCycleNs);

    /**
     * @brief Set duty cycle in nanoseconds
     */
    int setDutyCycleNS(int ns) const;

    /**
     * @brief Set PWM period in nanoseconds
     */
    int setPeriod(int ns) const;

    /**
     * @brief Disable PWM output
     */
    int disable();


private:
    /**
     * @brief Writes an integer value to one sysfs file.
     */
    int writeSYS(const std::string &filename, int value) const;

    int exportChannel(int channel);
    int waitForChannel() const;

    std::string chipPath;
    std::string pwmPath;

    // Records whether this object has enabled its PWM output.
    bool started = false;

    static constexpr int exportAttempts = 10;
    static constexpr int retryDelayUs = 100000;
};

#endif
