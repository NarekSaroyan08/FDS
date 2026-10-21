#include "facetracker/sdk.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <vector>

namespace {

int failures = 0;

void expect(bool condition, const char* description)
{
    if (!condition)
    {
        ++failures;
        std::cerr << "ОШИБКА: " << description << '\n';
    }
}

void testMotorDirection()
{
    using facetracker::MotorDirection;
    using facetracker::motorDirectionForFace;

    expect(motorDirectionForFace(209, 480) == MotorDirection::Up,
           "лицо выше центральной зоны: вверх");
    expect(motorDirectionForFace(271, 480) == MotorDirection::Down,
           "лицо ниже центральной зоны: вниз");
    expect(motorDirectionForFace(210, 480) == MotorDirection::Up,
           "верхняя граница зоны: вверх, как в исходной версии");
    expect(motorDirectionForFace(270, 480) == MotorDirection::Down,
           "нижняя граница зоны: вниз, как в исходной версии");
    expect(motorDirectionForFace(211, 480) == MotorDirection::Stop,
           "внутри верхней границы: стоп");
    expect(motorDirectionForFace(269, 480) == MotorDirection::Stop,
           "внутри нижней границы: стоп");
    expect(motorDirectionForFace(240, 480) == MotorDirection::Stop,
           "центр кадра: стоп");
    expect(motorDirectionForFace(10, 0) == MotorDirection::Stop,
           "некорректная высота кадра: стоп");
    expect(motorDirectionForFace(10, 480, -1) == MotorDirection::Stop,
           "некорректная мёртвая зона: стоп");
}

void testCalibrationInterpolation()
{
    using facetracker::CalibrationPoint;
    using facetracker::servoAnglesForPoint;

    const std::vector<CalibrationPoint> empty;
    const auto fallback = servoAnglesForPoint(empty, 320, 240);
    expect(fallback.x == 90 && fallback.y == 90,
           "пустая таблица: центральные углы");

    const std::vector<CalibrationPoint> one{{100, 200, 45, 120}};
    const auto single = servoAnglesForPoint(one, 100, 200);
    expect(single.x == 45 && single.y == 120,
           "одна точка: её углы");

    const std::vector<CalibrationPoint> two{{0, 0, 40, 60},
                                            {10, 0, 100, 120}};
    const auto middle = servoAnglesForPoint(two, 5, 0);
    expect(std::abs(middle.x - 70) <= 1 && std::abs(middle.y - 90) <= 1,
           "одинаковые расстояния: средние углы");

    const auto nearest = servoAnglesForPoint(two, 1, 0, 1);
    expect(nearest.x == 40 && nearest.y == 60,
           "один ближайший сосед");

    const std::vector<CalibrationPoint> weightedPoints{{0, 0, 0, 0},
                                                       {10, 0, 100, 100}};
    const auto weighted = servoAnglesForPoint(weightedPoints, 2, 0);
    expect(std::abs(weighted.x - 10) <= 1 &&
               std::abs(weighted.y - 10) <= 1,
           "вес 1/(расстояние + 1)^2 влияет на углы");

    const auto noNeighbours = servoAnglesForPoint(two, 1, 0, 0);
    expect(noNeighbours.x == 90 && noNeighbours.y == 90,
           "нулевое число соседей: центральные углы");
}

void testPulseLimiter()
{
    using facetracker::MotorDirection;
    using facetracker::PulseLimiter;
    using namespace std::chrono;

    PulseLimiter limiter(2, milliseconds(500));
    const auto start = PulseLimiter::Clock::time_point{};

    expect(!limiter.canPulse(MotorDirection::Stop, start),
           "команда стоп не создаёт импульс");
    expect(limiter.canPulse(MotorDirection::Up, start),
           "первый импульс разрешён");

    limiter.recordCompletedPulse(start + milliseconds(120));
    expect(!limiter.canPulse(MotorDirection::Down, start + milliseconds(619)),
           "пауза после импульса соблюдается");
    expect(limiter.canPulse(MotorDirection::Down, start + milliseconds(620)),
           "после паузы направление можно сменить");

    limiter.recordCompletedPulse(start + milliseconds(740));
    expect(limiter.limitReached() && limiter.pulseCount() == 2,
           "достигнут предел импульсов");
    expect(!limiter.canPulse(MotorDirection::Up, start + seconds(10)),
           "после предела новые импульсы запрещены");

    bool rejected = false;
    try
    {
        PulseLimiter invalid(0, milliseconds(500));
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    expect(rejected, "нулевой предел импульсов отклоняется");

    rejected = false;
    try
    {
        PulseLimiter invalid(2, milliseconds(-1));
    }
    catch (const std::invalid_argument&)
    {
        rejected = true;
    }
    expect(rejected, "отрицательная пауза отклоняется");
}

} // namespace

int main()
{
    testMotorDirection();
    testCalibrationInterpolation();
    testPulseLimiter();

    if (failures != 0)
    {
        std::cerr << "Тесты SDK: ошибок " << failures << '\n';
        return 1;
    }

    std::cout << "Тесты SDK: все проверки пройдены\n";
    return 0;
}
