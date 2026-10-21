#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include "facetracker/sdk.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <cmath>
#include <fstream>
#include <chrono>
#include <string>
#include <stdexcept>
#include <csignal>
#include <cstdint>

#define SERVO_X 17
#define SERVO_Y 27
#define MOTOR_DOWN_PIN 13
#define MOTOR_UP_PIN 5

constexpr auto MOTOR_FRAME_TIMEOUT = std::chrono::milliseconds(500);
constexpr auto MOTOR_FACE_GRACE = std::chrono::milliseconds(600);
constexpr int MOTOR_START_DEADBAND = 40;
constexpr int MOTOR_STOP_DEADBAND = 20;
constexpr double CAMERA_FPS = 60.0;

using facetracker::MotorDirection;
enum class MotorTarget { NoFace, Centered, Up, Down };

bool stopMotor(unsigned inactiveLevel)
{
    const int downResult = gpioWrite(MOTOR_DOWN_PIN, inactiveLevel);
    const int upResult = gpioWrite(MOTOR_UP_PIN, inactiveLevel);
    return downResult == 0 && upResult == 0;
}

bool setMotorDirection(MotorDirection direction, unsigned activeLevel,
                       const std::atomic<bool> &keepRunning,
                       volatile std::sig_atomic_t &stopRequested)
{
    const unsigned inactiveLevel = 1 - activeLevel;
    if (!stopMotor(inactiveLevel))
        return false;
    if (direction == MotorDirection::Stop)
        return true;

    // Сначала отпустить оба реле, затем включить нужное направление.
    gpioDelay(50000);
    if (!keepRunning || stopRequested)
        return true;
    if (gpioWrite(direction == MotorDirection::Up ? MOTOR_UP_PIN : MOTOR_DOWN_PIN,
                  activeLevel) != 0)
    {
        stopMotor(inactiveLevel);
        return false;
    }
    return true;
}

std::int64_t steadyClockMilliseconds()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

#ifndef CALIBRATION_FILE
#define CALIBRATION_FILE "data/fixed_to_servo.txt"
#endif
#ifndef CASCADE_FILE
#define CASCADE_FILE "data/haarcascade_frontalface_default.xml"
#endif


std::vector<facetracker::CalibrationPoint> LUT;

// ==========================
// GLOBAL FLAGS
std::atomic<bool> running(true);
std::atomic<bool> faceDetected(false);
std::atomic<MotorTarget> motorTarget(MotorTarget::NoFace);
std::atomic<std::int64_t> lastMotorFrameMs(-1);
std::atomic<bool> motorRunLimitReached(false);
volatile std::sig_atomic_t shutdownRequested = 0;

void handleSignal(int)
{
    shutdownRequested = 1;
}

// ==========================
// THREAD SAFE SERVO TARGETS
std::atomic<int> servoX(90);
std::atomic<int> servoY(90);

// ==========================
// SCAN TIME
double scanTime = 0;

// ==========================
// PID
struct PID
{
    double kp, ki, kd;

    double prevError = 0;
    double integral = 0;

    double update(double error)
    {
        integral += error;

        double derivative = error - prevError;

        prevError = error;

        return kp * error
             + ki * integral
             + kd * derivative;
    }
};

PID pidX{0.12, 0.0001, 0.08};
PID pidY{0.12, 0.0001, 0.08};

// ==========================
// LOAD LUT
void loadLUT()
{
    std::ifstream file(CALIBRATION_FILE);

    if (!file.is_open())
    {
        std::cout << "No fixed_to_servo.txt found\n";
        return;
    }

    facetracker::CalibrationPoint p;

    while (file >> p.px >> p.py >> p.sx >> p.sy)
    {
        LUT.push_back(p);
    }

    std::cout << "Loaded LUT: "
              << LUT.size()
              << std::endl;
}

// ==========================
int main(int argc, char *argv[])
{
    std::string fixedCameraDevice = "/dev/video0";
    bool motorEnabled = false;
    bool cameraProvided = false;
    bool relayPolarityProvided = false;
    unsigned motorActiveLevel = 1;
    try
    {
        for (int i = 1; i < argc; ++i)
        {
            const std::string arg = argv[i];
            if (arg == "--enable-motor")
                motorEnabled = true;
            else if (arg == "--relay-active-low" && !relayPolarityProvided)
            {
                motorActiveLevel = 0;
                relayPolarityProvided = true;
            }
            else if (arg == "--relay-active-high" && !relayPolarityProvided)
            {
                motorActiveLevel = 1;
                relayPolarityProvided = true;
            }
            else if (!cameraProvided && !arg.empty() && arg[0] != '-')
            {
                cameraProvided = true;
                if (std::all_of(arg.begin(), arg.end(),
                            [](char c) { return c >= '0' && c <= '9'; }))
                    fixedCameraDevice = "/dev/video" + arg;
                else
                    fixedCameraDevice = arg;
            }
            else
                throw std::invalid_argument("arguments");
        }
        if (fixedCameraDevice.empty() ||
            (relayPolarityProvided && !motorEnabled))
            throw std::invalid_argument("arguments");
    }
    catch (const std::exception &)
    {
        std::cerr << "Usage: facetracking [fixed_camera_device] "
                     "[--enable-motor [--relay-active-high|--relay-active-low]]\n";
        return 1;
    }

    // ==========================
    // REDUCE OPENCV THREAD ISSUES
    cv::setNumThreads(1);

    // ==========================
    // INIT PIGPIO
    gpioCfgSetInternals(gpioCfgGetInternals() | PI_CFG_NOSIGHANDLER);
    if (gpioInitialise() < 0)
    {
        std::cout << "pigpio failed\n";
        return -1;
    }
    std::signal(SIGINT, handleSignal);
    std::signal(SIGTERM, handleSignal);

    // ==========================
    // SAFE PWM FREQUENCY
    gpioSetPWMfrequency(SERVO_X, 50);
    gpioSetPWMfrequency(SERVO_Y, 50);

    // ==========================
    // LOAD LUT
    loadLUT();
    if (LUT.empty())
    {
        std::cerr << "No fixed-to-servo calibration points. "
                     "Run make calibrate first.\n";
        gpioTerminate();
        return 1;
    }

    // ==========================
    // Only the fixed camera supplies face positions. The moving camera
    // is the pan/tilt payload and is not opened during tracking.
    cv::VideoCapture cap;
    if (!cap.open(fixedCameraDevice, cv::CAP_V4L2))
    {
        std::cerr << "Cannot open fixed camera: " << fixedCameraDevice << "\n";
        gpioTerminate();
        return 1;
    }

    cap.set(cv::CAP_PROP_FOURCC,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, CAMERA_FPS);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    std::cout << "Fixed camera requested FPS: " << CAMERA_FPS
              << ", driver reports: " << cap.get(cv::CAP_PROP_FPS) << '\n';

    // ==========================
    // FACE CASCADE
    cv::CascadeClassifier face_cascade;

    if (!face_cascade.load(
        CASCADE_FILE))
    {
        std::cout << "Cascade load failed\n";
        gpioTerminate();
        return -1;
    }

    if (motorEnabled)
    {
        // Both inactive relay inputs must physically stop the motor.
        const unsigned inactiveLevel = 1 - motorActiveLevel;
        const unsigned pull = inactiveLevel ? PI_PUD_UP : PI_PUD_DOWN;
        gpioSetPullUpDown(MOTOR_DOWN_PIN, pull);
        gpioSetPullUpDown(MOTOR_UP_PIN, pull);
        const bool preset = stopMotor(inactiveLevel);
        const int downMode = gpioSetMode(MOTOR_DOWN_PIN, PI_OUTPUT);
        const int upMode = gpioSetMode(MOTOR_UP_PIN, PI_OUTPUT);
        const bool stopped = stopMotor(inactiveLevel);
        if (!preset || downMode < 0 || upMode < 0 || !stopped)
        {
            std::cerr << "Cannot initialise motor relay GPIO 13/5\n";
            gpioTerminate();
            return 1;
        }
    }

    // ==========================
    // SAFE CENTER
    gpioServo(SERVO_X, 1500);
    gpioServo(SERVO_Y, 1500);

    // ==========================
    // VISION THREAD
    std::thread visionThread([&]()
    {
        int lastFx = 320;
        int lastFy = 240;
        MotorDirection latchedMotorDirection = MotorDirection::Stop;
        std::int64_t lastDetectedFaceMs = -1;
        while (running && !shutdownRequested)
        {
            cv::Mat frame;

            if (!cap.read(frame))
                continue;

            // SAFETY
            if (frame.empty())
                continue;
            if (frame.cols != 640 || frame.rows != 480)
            {
                std::cerr << "Fixed camera must deliver 640x480 frames\n";
                running = false;
                break;
            }

            cv::Mat gray;
            cv::Mat small;

            cv::cvtColor(
                frame,
                gray,
                cv::COLOR_BGR2GRAY
            );

            cv::resize(
                gray,
                small,
                cv::Size(),
                0.5,
                0.5
            );

            std::vector<cv::Rect> faces;

            face_cascade.detectMultiScale(
                small,
                faces,
                1.2,
                3
            );

            std::string motorPreview = "STOP (no face)";
            MotorDirection motorDirection = MotorDirection::Stop;

            // ==========================
            // FACE FOUND
            if (faces.size() > 0)
            {
                faceDetected = true;
                const auto faceTime = steadyClockMilliseconds();
                if (lastDetectedFaceMs < 0 ||
                    faceTime - lastDetectedFaceMs > MOTOR_FACE_GRACE.count())
                    latchedMotorDirection = MotorDirection::Stop;
                lastDetectedFaceMs = faceTime;

                cv::Rect f = faces[0];

                int fx =
                    (f.x + f.width / 2) * 2;

                int fy =
                    (f.y + f.height / 2) * 2;

                motorDirection = facetracker::motorDirectionWithHysteresis(
                    fy, frame.rows, latchedMotorDirection,
                    MOTOR_START_DEADBAND, MOTOR_STOP_DEADBAND);
                latchedMotorDirection = motorDirection;
                switch (motorDirection)
                {
                case MotorDirection::Up:
                    motorPreview = "UP";
                    break;
                case MotorDirection::Down:
                    motorPreview = "DOWN";
                    break;
                case MotorDirection::Stop:
                    motorPreview = "STOP (centered)";
                    break;
                }

                // ==========================
                // PREDICTION
                int predFx =
                    fx + (fx - lastFx) * 0.5;

                int predFy =
                    fy + (fy - lastFy) * 0.5;

                lastFx = fx;
                lastFy = fy;

                const auto target = facetracker::servoAnglesForPoint(
                    LUT, predFx, predFy);
                servoX = target.x;
                servoY = target.y;

                // ==========================
                // DRAW
                cv::circle(
                    frame,
                    {fx, fy},
                    5,
                    {0,255,0},
                    -1
                );
            }
            else
            {
                // ==========================
                // NO FACE
                faceDetected = false;

                scanTime += 0.03;

                double waveX =
                    sin(scanTime);

                double waveY =
                    cos(scanTime * 0.7);

                servoX =
                    90 + waveX * 40;

                servoY =
                    90 + waveY * 20;

            }

            motorTarget = faces.empty() ? MotorTarget::NoFace :
                          motorDirection == MotorDirection::Up ? MotorTarget::Up :
                          motorDirection == MotorDirection::Down ? MotorTarget::Down :
                          MotorTarget::Centered;
            lastMotorFrameMs = steadyClockMilliseconds();

            // ==========================
            // Show the direction and whether relay output is enabled.
            cv::line(frame, {0, frame.rows / 2},
                     {frame.cols - 1, frame.rows / 2}, {0, 255, 255}, 1);
            const std::string motorState = !motorEnabled ? "relay disabled" :
                                           motorRunLimitReached ? "run limit" :
                                           "relay enabled";
            cv::putText(frame, "Motor: " + motorPreview + " (" + motorState + ")",
                        {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 0.55,
                        {0, 0, 255}, 2);

            // SHOW WINDOW
            cv::imshow(
                "PRO Tracker",
                frame
            );

            char key =
                cv::waitKey(1);

            if (key == 'q')
            {
                running = false;
            }
        }
    });

    // ==========================
    // SERVO THREAD
    std::thread servoThread([&]()
    {
        double outX = 90;
        double outY = 90;

        while (running && !shutdownRequested)
        {
            // PID ERRORS
            double errX =
                servoX - outX;

            double errY =
                servoY - outY;

            outX += pidX.update(errX);
            outY += pidY.update(errY);

            // SAFE LIMITS
            outX = std::clamp(
                outX,
                20.0,
                160.0
            );

            outY = std::clamp(
                outY,
                20.0,
                160.0
            );

            // SAFE PULSES
            int pulseX =
                1000 + outX * 5;

            int pulseY =
                1000 + outY * 5;

            gpioServo(
                SERVO_X,
                pulseX
            );

            gpioServo(
                SERVO_Y,
                pulseY
            );

            std::this_thread::sleep_for(
                std::chrono::milliseconds(15)
            );
        }
    });

    // Реле удерживается включённым, пока лицо вне центра и поступают кадры.
    std::thread motorThread([&]()
    {
        if (!motorEnabled)
            return;

        facetracker::MotorRunGuard guard(std::chrono::seconds(5),
                                        MOTOR_FACE_GRACE);
        MotorDirection applied = MotorDirection::Stop;
        while (running && !shutdownRequested)
        {
            const auto now = facetracker::MotorRunGuard::Clock::now();
            const auto frameTime = lastMotorFrameMs.load();
            const auto age = steadyClockMilliseconds() - frameTime;
            const bool frameFresh = frameTime >= 0 && age >= 0 &&
                                    age <= MOTOR_FRAME_TIMEOUT.count();
            const auto target = motorTarget.load();
            const auto requested = target == MotorTarget::Up ? MotorDirection::Up :
                                   target == MotorTarget::Down ? MotorDirection::Down :
                                   MotorDirection::Stop;
            const auto next = guard.update(requested, frameFresh,
                                           target != MotorTarget::NoFace, now);
            motorRunLimitReached = guard.limitReached();

            if (next != applied)
            {
                if (!setMotorDirection(next, motorActiveLevel,
                                       running, shutdownRequested))
                {
                    std::cerr << "Motor relay GPIO failed; disconnect motor power\n";
                    running = false;
                    break;
                }
                applied = next;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
        if (!stopMotor(1 - motorActiveLevel))
            std::cerr << "Motor relay GPIO stop failed; disconnect motor power\n";
    });

    // ==========================
    // WAIT
    visionThread.join();
    servoThread.join();
    motorThread.join();

    // ==========================
    // STOP EVERYTHING
    gpioServo(SERVO_X, 0);
    gpioServo(SERVO_Y, 0);
    if (motorEnabled && !stopMotor(1 - motorActiveLevel))
        std::cerr << "Motor relay GPIO stop failed; disconnect motor power\n";

    gpioTerminate();

    return 0;
}
