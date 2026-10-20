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
