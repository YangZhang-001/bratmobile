#pragma once

#include "targetloc.h"

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <cstddef>
#include <vector>

/**
 * Passes a TargetLoc result back to the main thread.
 *
 * TargetLoc calls this from its detection thread, so stopping the
 * cameras or changing the navigation state is left to main().
 */
class TargetLocReceiver final : public TargetLoc::DetectionInterface
{
  public:
    void newTargetDetected(const cv::Point2f target) override;

    // Wait for a target, or return false if the wait times out.
    bool waitForTarget(cv::Point2f &target,
                       std::chrono::milliseconds timeout);

    // Used before TargetLoc is stopped during the phase change.
    void close();

  private:
    std::mutex receiverMutex;
    std::condition_variable targetAvailable;

    cv::Point2f lockedTarget{0.0F, 0.0F};

    bool accepting = true;
    bool targetLocked = false;
};


/**
 * Checks a few TargetLoc results before passing one to the basic receiver.
 */
class StableTargetLocReceiver final : public TargetLoc::DetectionInterface
{
  public:
    StableTargetLocReceiver(std::size_t requiredSamples,
                            float maxSampleSeparationM);

    void newTargetDetected(const cv::Point2f target) override;

    bool waitForTarget(cv::Point2f &target,
                       std::chrono::milliseconds timeout);

    void close();

  private:
    bool isConsistentWithSamples(const cv::Point2f &target) const;
    cv::Point2f calculateMean() const;

    const std::size_t requiredSamples;
    const float maxSampleSeparationM;

    std::mutex sampleMutex;
    std::vector<cv::Point2f> samples;
    bool acceptingSamples = true;

    TargetLocReceiver receiver;
};
