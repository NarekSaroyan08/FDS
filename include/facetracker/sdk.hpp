#pragma once

#include <chrono>
#include <cstddef>
#include <vector>

namespace facetracker {

struct CalibrationPoint {
    int px;
    int py;
    int sx;
    int sy;
};

struct ServoAngles {
    int x;
    int y;
};

enum class MotorDirection { Stop, Up, Down };

// Начало координат кадра находится в левом верхнем углу.
MotorDirection motorDirectionForFace(int faceY, int frameHeight,
                                    int deadbandPixels = 30) noexcept;
MotorDirection motorDirectionWithHysteresis(
    int faceY, int frameHeight, MotorDirection previous,
    int startDeadbandPixels = 40, int stopDeadbandPixels = 20) noexcept;

// Непрерывное движение, пока кадры свежие; после лимита нужно снова увидеть центр.
class MotorRunGuard {
public:
    using Clock = std::chrono::steady_clock;

    explicit MotorRunGuard(std::chrono::milliseconds maximumRun =
                              std::chrono::seconds(5),
                           std::chrono::milliseconds missingFaceGrace =
                              std::chrono::milliseconds(600));

    MotorDirection update(MotorDirection requested, bool frameFresh,
                          bool faceVisible, Clock::time_point now) noexcept;
    bool limitReached() const noexcept;

private:
    std::chrono::milliseconds maximumRun_;
    std::chrono::milliseconds missingFaceGrace_;
    Clock::time_point runStartedAt_ = Clock::time_point::min();
    Clock::time_point lastFaceSeenAt_ = Clock::time_point::min();
    MotorDirection lastDirection_ = MotorDirection::Stop;
    bool limitReached_ = false;
};

// Интерполяция ближайших точек с весом 1 / (расстояние + 1)^2.
// При пустой таблице или nearestCount == 0 возвращает {90, 90}.
ServoAngles servoAnglesForPoint(const std::vector<CalibrationPoint>& points,
                               int x, int y, std::size_t nearestCount = 5);

class PulseLimiter {
public:
    using Clock = std::chrono::steady_clock;

    explicit PulseLimiter(int maximumPulses = 20,
                          std::chrono::milliseconds cooldown =
                              std::chrono::milliseconds(500));

    bool canPulse(MotorDirection direction, Clock::time_point now) const noexcept;
    void recordCompletedPulse(Clock::time_point finishedAt) noexcept;
    bool limitReached() const noexcept;
    int pulseCount() const noexcept;

private:
    int maximumPulses_;
    std::chrono::milliseconds cooldown_;
    int pulseCount_ = 0;
    Clock::time_point nextPulse_ = Clock::time_point::min();
};

} // namespace facetracker
