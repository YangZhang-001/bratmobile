#include "Pwm.h"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <unistd.h>
#include <fstream>

PWM::~PWM()
{
    disable();
}

int PWM::writeSYS(const std::string &filename, const int value) const
{
    const int fd = open(filename.c_str(), O_WRONLY);

    if (fd < 0) {
        std::cerr << "Could not open " << filename << ": "
                  << std::strerror(errno) << std::endl;
        return -1;
    }

    const std::string text = std::to_string(value);
    const ssize_t written = write(fd, text.c_str(), text.size());
    const int writeError = errno;

    if (close(fd) < 0) {
        std::cerr << "Could not close " << filename << ": "
                  << std::strerror(errno) << std::endl;
        return -1;
    }

    if (written != static_cast<ssize_t>(text.size())) {
        std::cerr << "Could not write " << filename;

        if (written < 0)
            std::cerr << ": " << std::strerror(writeError);

        std::cerr << std::endl;
        return -1;
    }

    return 0;
}

int PWM::exportChannel(const int channel)
{
    // Reuse a channel exported by an earlier program run.
    if (access(pwmPath.c_str(), F_OK) == 0)
        return 0;

    if (writeSYS(chipPath + "/export", channel) == 0)
        return 0;

    // Another process may have exported it at the same time.
    return access(pwmPath.c_str(), F_OK) == 0 ? 0 : -1;
}

int PWM::waitForChannel() const
{
    const std::string periodPath =
        pwmPath + "/period";
    const std::string dutyCyclePath =
        pwmPath + "/duty_cycle";
    const std::string enablePath =
        pwmPath + "/enable";

    //wait until udev has made the exported channel usable
    for (int attempt = 0; attempt <= exportAttempts; ++attempt) {
        if (access(periodPath.c_str(), R_OK | W_OK) == 0
            && access(dutyCyclePath.c_str(), W_OK) == 0
            && access(enablePath.c_str(), R_OK | W_OK) == 0) {
            return 0;
        }
        usleep(retryDelayUs);
    }

    std::cerr << "Timeout waiting for usable PWM channel" << pwmPath << std::endl;
    return -1;
}

int PWM::start(const int channel,
               const int chip,
               const int periodNs,
               const int dutyCycleNs)
{
    if (started)
        return 0;

    if (periodNs <= 0
        || dutyCycleNs < 0
        || dutyCycleNs > periodNs) {
        std::cerr << "Invalid PWM period or duty cycle." << std::endl;
        return -1;
    }

    chipPath = "/sys/class/pwm/pwmchip" + std::to_string(chip);
    pwmPath = chipPath + "/pwm" + std::to_string(channel);

    if (access(chipPath.c_str(), F_OK) != 0) {
        std::cerr << "PWM chip does not exist: "
                  << chipPath << std::endl;
        return -1;
    }

    if (exportChannel(channel) < 0 || waitForChannel() < 0)
        return -1;

    // Configure the channel while its output is disabled.
    if (disable() < 0)
        return -1;

    // Some PWM drivers reject a smaller period while duty is larger.
    //if (setDutyCycleNS(0) < 0)
    //    return -1;

    //rock 5 PWM channel has a zero period.
    int currentPeriod = -1;

    {
        std::ifstream periodFile(pwmPath + "/period");

        if (!(periodFile >> currentPeriod)) {
            std::cerr << "Could not read "
                        << pwmPath << "/period"
                        << std::endl;
            return -1;
        }
    }

    if (currentPeriod < 0) {
        std::cerr << "Invalid PWM period state: "
                  << currentPeriod << std::endl;
        return -1;
    }

    // configured channel may need its duty cleared before changing to a smaller period
    if (currentPeriod > 0
        && setDutyCycleNS(0) < 0) {
        return -1;
    }

    if (setPeriod(periodNs) < 0)
        return -1;

    if (setDutyCycleNS(dutyCycleNs) < 0)
        return -1;

    // Enable only after period and duty cycle are configured.
    if (writeSYS(pwmPath + "/enable", 1) < 0) {
        disable();
        return -1;
    }

    started = true;
    return 0;
}

int PWM::setDutyCycleNS(const int ns) const
{
    if (pwmPath.empty() || ns < 0)
        return -1;

    return writeSYS(pwmPath + "/duty_cycle", ns);
}

int PWM::setPeriod(const int ns) const
{
    if (pwmPath.empty() || ns <= 0)
        return -1;

    return writeSYS(pwmPath + "/period", ns);
}

int PWM::disable()
{
    const std::string enablePath = pwmPath + "/enable";

    // A default-constructed or partially started object is safe to destroy.
    if (pwmPath.empty() || access(enablePath.c_str(), F_OK) != 0)
        return 0;

    int enabled = -1;

    {
        std::ifstream enableFile(enablePath);

        if (!(enableFile >> enabled)) {
            std::cerr << "Could not read " << enablePath
                        << std::endl;
            return -1;
        }
    }

    // if PWM drivers reject writing 0 when already disabled.
    if (enabled == 0) {
        started = false;
        return 0;
    }

    if (enabled != 1) {
        std::cerr << "Unexpected PWM enable state in "
                << enablePath << ": " << enabled
                << std::endl;
        return -1;
    }

    const int result = writeSYS(enablePath, 0);

    // Keep the active state when disabling fails so it can be retried.
    if (result == 0)
        started = false;

    return result;
}