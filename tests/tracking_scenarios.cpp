#include "facetracker/sdk.hpp"

#include <chrono>
#include <iostream>

namespace {

using facetracker::MotorDirection;
using facetracker::MotorRunGuard;
using namespace std::chrono;

int failures = 0;

void expect(MotorDirection actual, MotorDirection expected,
            const char *description)
{
    if (actual != expected)
    {
        ++failures;
        std::cerr << "ОШИБКА: " << description << '\n';
    }
}

class TrackingScenario {
public:
    TrackingScenario() : guard_(seconds(5), milliseconds(1500), milliseconds(300)) {}

    MotorDirection frame(int elapsedMs, int faceY, bool faceVisible = true,
                         bool frameFresh = true)
    {
        if (faceVisible)
        {
            direction_ = facetracker::motorDirectionWithHysteresis(
                faceY, 480, direction_);
        }
        return guard_.update(faceVisible ? direction_ : MotorDirection::Stop,
                             frameFresh, faceVisible,
                             MotorRunGuard::Clock::time_point{} +
                                 milliseconds(elapsedMs));
    }

    bool limitReached() const { return guard_.limitReached(); }

private:
    MotorRunGuard guard_;
    MotorDirection direction_ = MotorDirection::Stop;
};

void testFaceMovesAcrossCenter()
{
    TrackingScenario scenario;
    expect(scenario.frame(0, 180), MotorDirection::Stop,
           "первый кадр выше центра: ожидание устойчивой команды");
    expect(scenario.frame(150, 180), MotorDirection::Stop,
           "через 150 мс реле ещё выключено");
    expect(scenario.frame(300, 180), MotorDirection::Up,
           "после 300 мс мотор движется вверх");
    expect(scenario.frame(400, 0, false), MotorDirection::Up,
           "пропуск распознавания не прерывает движение");
    expect(scenario.frame(1000, 0, false), MotorDirection::Up,
           "серия кратких пропусков не создаёт импульсов");
    expect(scenario.frame(1100, 180), MotorDirection::Up,
           "после возврата лица реле остаётся включённым");
    expect(scenario.frame(1200, 240), MotorDirection::Stop,
           "в центре мотор останавливается");
    expect(scenario.frame(1300, 300), MotorDirection::Stop,
           "перед движением вниз нужна устойчивая команда");
    expect(scenario.frame(1600, 300), MotorDirection::Down,
           "ниже центра мотор движется вниз");
    expect(scenario.frame(1700, 300, true, false), MotorDirection::Stop,
           "при пропаже кадров реле сразу выключается");
    expect(scenario.frame(1800, 300), MotorDirection::Stop,
           "после восстановления кадров направление выдерживается заново");
    expect(scenario.frame(2100, 300), MotorDirection::Down,
           "после выдержки мотор снова движется вниз");
}

void testRunLimitRequiresCenter()
{
    TrackingScenario scenario;
    expect(scenario.frame(0, 180), MotorDirection::Stop,
           "стартовая выдержка перед включением");
    expect(scenario.frame(300, 180), MotorDirection::Up,
           "мотор включился после выдержки");
    expect(scenario.frame(5299, 180), MotorDirection::Up,
           "до пяти секунд непрерывное движение разрешено");
    expect(scenario.frame(5300, 180), MotorDirection::Stop,
           "через пять секунд мотор остановлен");
    if (!scenario.limitReached())
    {
        ++failures;
        std::cerr << "ОШИБКА: лимит работы должен быть активен\n";
    }
    expect(scenario.frame(5500, 0, false), MotorDirection::Stop,
           "потеря лица не снимает лимит");
    expect(scenario.frame(5600, 180), MotorDirection::Stop,
           "лицо вне центра не запускает мотор после лимита");
    expect(scenario.frame(5700, 240), MotorDirection::Stop,
           "лицо в центре сбрасывает лимит");
    if (scenario.limitReached())
    {
        ++failures;
        std::cerr << "ОШИБКА: лимит должен быть снят в центре\n";
    }
    expect(scenario.frame(5800, 180), MotorDirection::Stop,
           "повторный старт снова требует выдержки");
    expect(scenario.frame(6100, 180), MotorDirection::Up,
           "после центра и выдержки движение снова разрешено");
}

} // namespace

int main()
{
    testFaceMovesAcrossCenter();
    testRunLimitRequiresCenter();
    if (failures != 0)
    {
        std::cerr << "Сценарные unit-тесты: ошибок " << failures << '\n';
        return 1;
    }
    std::cout << "Сценарные unit-тесты: все проверки пройдены\n";
    return 0;
}
