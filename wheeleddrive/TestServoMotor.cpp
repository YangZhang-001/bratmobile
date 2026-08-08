#include "Driving.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace
{
volatile std::sig_atomic_t stopRequested = 0;

constexpr float maximumTestSpeed = 0.10F;
constexpr float defaultDurationSeconds = 2.0F;
constexpr float maximumDurationSeconds = 5.0F;

void requestStop(int)
{
    stopRequested = 1;
}

bool parseFloat(const char *text, float &value)
{
    errno = 0;
    char *end = nullptr;

    value = std::strtof(text, &end);

    return errno == 0
           && end != text
           && *end == '\0'
           && std::isfinite(value);
}
}

int main(int argc, char *argv[])
{
    if (argc < 3 || argc > 4) {
        std::cerr
            << "Usage: " << argv[0]
            << " <left_speed> <right_speed> [duration_seconds]\n"
            << "Safe test limits: speed +/-" << maximumTestSpeed
            << ", duration up to " << maximumDurationSeconds
            << " seconds.\n";

        return EXIT_FAILURE;
    }

    float leftSpeed = 0.0F;
    float rightSpeed = 0.0F;
    float durationSeconds = defaultDurationSeconds;

    if (!parseFloat(argv[1], leftSpeed)
        || !parseFloat(argv[2], rightSpeed)
        || (argc == 4
            && !parseFloat(argv[3], durationSeconds))) {
        std::cerr << "Invalid numeric argument.\n";
        return EXIT_FAILURE;
    }

    if (std::fabs(leftSpeed) > maximumTestSpeed
        || std::fabs(rightSpeed) > maximumTestSpeed) {
        std::cerr
            << "Requested speed exceeds the safe test limit of +/-"
            << maximumTestSpeed << ".\n";

        return EXIT_FAILURE;
    }

    if (durationSeconds <= 0.0F
        || durationSeconds > maximumDurationSeconds) {
        std::cerr
            << "Duration must be greater than 0 and no more than "
            << maximumDurationSeconds << " seconds.\n";

        return EXIT_FAILURE;
    }

    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);

    Driving driving;

    try {
        std::cerr
            << "Starting motor control at neutral speed... ";

        driving.start();

        std::cerr << "SUCCESS!\n";

        std::cout
            << "Setting speeds - Left: " << leftSpeed
            << ", Right: " << rightSpeed
            << ", Duration: " << durationSeconds
            << " seconds\n";

        if (driving.setMotorSpeeds(
                leftSpeed,
                rightSpeed) < 0) {
            std::cerr << "Could not set motor speeds.\n";
            driving.stop();
            return EXIT_FAILURE;
        }

        const auto duration =
            std::chrono::duration<float>(durationSeconds);

        const auto deadline =
            std::chrono::steady_clock::now() + duration;

        while (!stopRequested
               && std::chrono::steady_clock::now()
                      < deadline) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(50));
        }

        std::cerr << "Stopping motor control...\n";
        driving.stop();
    }
    catch (const std::exception &error) {
        std::cerr
            << "Motor test failed: "
            << error.what() << '\n';

        driving.stop();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}