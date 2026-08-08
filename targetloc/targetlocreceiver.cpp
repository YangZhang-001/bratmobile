#include "targetlocreceiver.h"

#include <cmath>
#include <stdexcept>

void TargetLocReceiver::newTargetDetected(const cv::Point2f target)
{
    // this should already be a calibrated TargetLoc result, but avoid
    // passing a broken value into navigation.
    if (!std::isfinite(target.x) || !std::isfinite(target.y))
    {
        return;
    }

    {
        std::lock_guard<std::mutex> guard(receiverMutex);

        if (!accepting || targetLocked)
        {
            return;
        }

        // start with the first valid result. A stability check can be
        // added here once the handover path is working.
        lockedTarget = target;
        targetLocked = true;
        accepting = false;
    }

    targetAvailable.notify_all();
}

bool TargetLocReceiver::waitForTarget(
    cv::Point2f &target,
    std::chrono::milliseconds timeout)
{
    std::unique_lock<std::mutex> lock(receiverMutex);

    const bool finished = targetAvailable.wait_for(
        lock,
        timeout,
        [this] {
            return targetLocked || !accepting;
        });

    if (!finished || !targetLocked)
    {
        return false;
    }

    target = lockedTarget;
    return true;
}

void TargetLocReceiver::close()
{
    {
        std::lock_guard<std::mutex> guard(receiverMutex);
        accepting = false;
    }

    targetAvailable.notify_all();
}

StableTargetLocReceiver::StableTargetLocReceiver(
    std::size_t requiredSamples, float maxSampleSeparationM)
    : requiredSamples(requiredSamples),
      maxSampleSeparationM(maxSampleSeparationM)
{
    if (requiredSamples == 0)
        throw std::invalid_argument("At least one sample is required");

    if (!std::isfinite(maxSampleSeparationM)
        || maxSampleSeparationM < 0.0F)
        throw std::invalid_argument("Invalid sample separation");

    samples.reserve(requiredSamples);
}

void StableTargetLocReceiver::newTargetDetected(
    const cv::Point2f target)
{
    // Filter the readings before using the basic receiver.
    if (!std::isfinite(target.x) || !std::isfinite(target.y))
        return;

    std::lock_guard<std::mutex> guard(sampleMutex);

    if (!acceptingSamples)
        return;

    // A large change starts a new group.
    if (!isConsistentWithSamples(target))
        samples.clear();

    samples.push_back(target);

    if (samples.size() < requiredSamples)
        return;

    acceptingSamples = false;
    receiver.newTargetDetected(calculateMean());
}

bool StableTargetLocReceiver::waitForTarget(
    cv::Point2f &target, std::chrono::milliseconds timeout)
{
    return receiver.waitForTarget(target, timeout);
}

void StableTargetLocReceiver::close()
{
    {
        std::lock_guard<std::mutex> guard(sampleMutex);
        acceptingSamples = false;
    }

    receiver.close();
}

bool StableTargetLocReceiver::isConsistentWithSamples(
    const cv::Point2f &target) const
{
    const float limit =
        maxSampleSeparationM * maxSampleSeparationM;

    for (const cv::Point2f &sample : samples)
    {
        const float dx = target.x - sample.x;
        const float dy = target.y - sample.y;

        if (dx * dx + dy * dy > limit)
            return false;
    }

    return true;
}

cv::Point2f StableTargetLocReceiver::calculateMean() const
{
    cv::Point2f mean(0.0F, 0.0F);

    for (const cv::Point2f &sample : samples)
        mean += sample;

    return mean / static_cast<float>(samples.size());
}
