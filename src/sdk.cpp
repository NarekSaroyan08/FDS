#include "facetracker/sdk.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace facetracker {

MotorDirection motorDirectionForFace(int faceY, int frameHeight,
                                    int deadbandPixels) noexcept
{
    if (frameHeight <= 0 || deadbandPixels < 0)
        return MotorDirection::Stop;

    const int errorY = faceY - frameHeight / 2;
    if (errorY > -deadbandPixels && errorY < deadbandPixels)
        return MotorDirection::Stop;
    return errorY < 0 ? MotorDirection::Up : MotorDirection::Down;
}

MotorDirection motorDirectionWithHysteresis(
    int faceY, int frameHeight, MotorDirection previous,
    int startDeadbandPixels, int stopDeadbandPixels) noexcept
{
    if (frameHeight <= 0 || startDeadbandPixels < stopDeadbandPixels ||
        stopDeadbandPixels < 0)
        return MotorDirection::Stop;

    const int deadband = previous == MotorDirection::Stop
                             ? startDeadbandPixels : stopDeadbandPixels;
    const auto next = motorDirectionForFace(faceY, frameHeight, deadband);
    if (previous != MotorDirection::Stop && next != previous)
        return MotorDirection::Stop;
    return next;
}

MotorRunGuard::MotorRunGuard(std::chrono::milliseconds maximumRun,
                           std::chrono::milliseconds missingFaceGrace,
                           std::chrono::milliseconds startDelay)
    : maximumRun_(maximumRun), missingFaceGrace_(missingFaceGrace),
      startDelay_(startDelay)
{
    if (maximumRun.count() <= 0 || missingFaceGrace.count() < 0 ||
        startDelay.count() < 0)
        throw std::invalid_argument("Некорректный предел непрерывной работы мотора");
}

MotorDirection MotorRunGuard::update(MotorDirection requested, bool frameFresh,
                                    bool faceVisible, Clock::time_point now) noexcept
{
    if (!frameFresh)
    {
        activeDirection_ = MotorDirection::Stop;
        pendingDirection_ = MotorDirection::Stop;
        return MotorDirection::Stop;
    }
    if (faceVisible && requested == MotorDirection::Stop)
    {
        runStartedAt_ = Clock::time_point::min();
        lastDirection_ = MotorDirection::Stop;
        activeDirection_ = MotorDirection::Stop;
        pendingDirection_ = MotorDirection::Stop;
        limitReached_ = false;
        return MotorDirection::Stop;
    }
    if (faceVisible)
    {
        lastFaceSeenAt_ = now;
        lastDirection_ = requested;
    }
    else if (lastDirection_ != MotorDirection::Stop &&
             lastFaceSeenAt_ != Clock::time_point::min() &&
             now - lastFaceSeenAt_ <= missingFaceGrace_)
    {
        requested = lastDirection_;
    }
    else
    {
        activeDirection_ = MotorDirection::Stop;
        pendingDirection_ = MotorDirection::Stop;
        return MotorDirection::Stop;
    }

    if (limitReached_)
        return MotorDirection::Stop;
    if (runStartedAt_ != Clock::time_point::min() &&
        now - runStartedAt_ >= maximumRun_)
    {
        limitReached_ = true;
        activeDirection_ = MotorDirection::Stop;
        pendingDirection_ = MotorDirection::Stop;
        return MotorDirection::Stop;
    }
    if (requested == activeDirection_)
        return activeDirection_;
    if (requested != pendingDirection_)
    {
        pendingDirection_ = requested;
        pendingSince_ = now;
    }
    activeDirection_ = MotorDirection::Stop;
    if (now - pendingSince_ < startDelay_)
        return MotorDirection::Stop;

    activeDirection_ = requested;
    pendingDirection_ = MotorDirection::Stop;
    if (runStartedAt_ == Clock::time_point::min())
        runStartedAt_ = now;
    return activeDirection_;
}

bool MotorRunGuard::limitReached() const noexcept
{
    return limitReached_;
}

ServoAngles servoAnglesForPoint(const std::vector<CalibrationPoint>& points,
                               int x, int y, std::size_t nearestCount)
{
    if (points.empty() || nearestCount == 0)
        return {90, 90};

    std::vector<std::pair<double, std::size_t>> distances;
    distances.reserve(points.size());
    for (std::size_t index = 0; index < points.size(); ++index)
    {
        const double dx = static_cast<double>(points[index].px) - x;
        const double dy = static_cast<double>(points[index].py) - y;
        distances.emplace_back(std::hypot(dx, dy), index);
    }

    std::sort(distances.begin(), distances.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });

    const std::size_t count = std::min(nearestCount, distances.size());
    double weightSum = 0.0;
    double xSum = 0.0;
    double ySum = 0.0;

    for (std::size_t i = 0; i < count; ++i)
    {
        const double shiftedDistance = distances[i].first + 1.0;
        const double weight = 1.0 / (shiftedDistance * shiftedDistance);
        const auto& point = points[distances[i].second];
        weightSum += weight;
        xSum += point.sx * weight;
        ySum += point.sy * weight;
    }

    return {static_cast<int>(xSum / weightSum),
            static_cast<int>(ySum / weightSum)};
}

PulseLimiter::PulseLimiter(int maximumPulses, std::chrono::milliseconds cooldown)
    : maximumPulses_(maximumPulses), cooldown_(cooldown)
{
    if (maximumPulses <= 0 || cooldown.count() < 0)
        throw std::invalid_argument("Некорректные ограничения импульсов мотора");
}

bool PulseLimiter::canPulse(MotorDirection direction,
                            Clock::time_point now) const noexcept
{
    return direction != MotorDirection::Stop && !limitReached() &&
           now >= nextPulse_;
}

void PulseLimiter::recordCompletedPulse(Clock::time_point finishedAt) noexcept
{
    if (limitReached())
        return;
    ++pulseCount_;
    nextPulse_ = finishedAt + cooldown_;
}

bool PulseLimiter::limitReached() const noexcept
{
    return pulseCount_ >= maximumPulses_;
}

int PulseLimiter::pulseCount() const noexcept
{
    return pulseCount_;
}

} // namespace facetracker
