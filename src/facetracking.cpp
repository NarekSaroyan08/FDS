#include <opencv2/opencv.hpp>
#include <pigpio.h>

#include <iostream>
#include <vector>
#include <algorithm>
#include <thread>
#include <atomic>
#include <cmath>
#include <fstream>
#include <chrono>

#define SERVO_X 17
#define SERVO_Y 27

#ifndef CALIBRATION_FILE
#define CALIBRATION_FILE "data/calibration.txt"
#endif
#ifndef CASCADE_FILE
#define CASCADE_FILE "data/haarcascade_frontalface_default.xml"
#endif

#define MOTOR_IN1 13
#define MOTOR_IN2 5

// ==========================
// CALIBRATION STRUCT
struct CalibrationPoint
{
    int px, py;
    int sx, sy;
};

std::vector<CalibrationPoint> LUT;

// ==========================
// GLOBAL FLAGS
std::atomic<bool> running(true);
std::atomic<bool> faceDetected(false);

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
        std::cout << "No calibration.txt found\n";
        return;
    }

    CalibrationPoint p;

    while (file >> p.px >> p.py >> p.sx >> p.sy)
    {
        LUT.push_back(p);
    }

    std::cout << "Loaded LUT: "
              << LUT.size()
              << std::endl;
}

// ==========================
// LUT INTERPOLATION
void getServoFromLUT(
    int fx,
    int fy,
    int &sx,
    int &sy
)
{
    // SAFETY
    if (LUT.empty())
    {
        sx = 90;
        sy = 90;
        return;
    }

    std::vector<
        std::pair<double, CalibrationPoint>
    > dist;

    for (auto &p : LUT)
    {
        double dx = p.px - fx;
        double dy = p.py - fy;

        double d = sqrt(dx * dx + dy * dy);

        dist.push_back({d, p});
    }

    std::sort(
        dist.begin(),
        dist.end(),
        [](auto &a, auto &b)
        {
            return a.first < b.first;
        }
    );

    int k = std::min(
        5,
        (int)dist.size()
    );

    double sumW = 0;
    double sumX = 0;
    double sumY = 0;

    for (int i = 0; i < k; i++)
    {
        double d = dist[i].first + 1.0;

        double w = 1.0 / (d * d);

        sumX += dist[i].second.sx * w;
        sumY += dist[i].second.sy * w;

        sumW += w;
    }

    if (sumW == 0)
    {
        sx = 90;
        sy = 90;
        return;
    }

    sx = int(sumX / sumW);
    sy = int(sumY / sumW);
}

// ==========================
int main()
{
    // ==========================
    // REDUCE OPENCV THREAD ISSUES
    cv::setNumThreads(1);

    // ==========================
    // INIT PIGPIO
    if (gpioInitialise() < 0)
    {
        std::cout << "pigpio failed\n";
        return -1;
    }

    // ==========================
    // GPIO MODES
    gpioSetMode(MOTOR_IN1, PI_OUTPUT);
    gpioSetMode(MOTOR_IN2, PI_OUTPUT);

    // ==========================
    // SAFE PWM FREQUENCY
    gpioSetPWMfrequency(SERVO_X, 50);
    gpioSetPWMfrequency(SERVO_Y, 50);

    // ==========================
    // LOAD LUT
    loadLUT();

    // ==========================
    // CAMERA
    cv::VideoCapture cap(0, cv::CAP_V4L2);

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    if (!cap.isOpened())
    {
        std::cout << "Camera failed\n";
        return -1;
    }

    // ==========================
    // FACE CASCADE
    cv::CascadeClassifier face_cascade;

    if (!face_cascade.load(
        CASCADE_FILE))
    {
        std::cout << "Cascade load failed\n";
        return -1;
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

        while (running)
        {
            cv::Mat frame;

            if (!cap.read(frame))
                continue;

            // SAFETY
            if (frame.empty())
                continue;

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

            // ==========================
            // FACE FOUND
            if (faces.size() > 0)
            {
                faceDetected = true;

                cv::Rect f = faces[0];

                int fx =
                    (f.x + f.width / 2) * 2;

                int fy =
                    (f.y + f.height / 2) * 2;

                // ==========================
                // PREDICTION
                int predFx =
                    fx + (fx - lastFx) * 0.5;

                int predFy =
                    fy + (fy - lastFy) * 0.5;

                lastFx = fx;
                lastFy = fy;

                int baseX;
                int baseY;

                getServoFromLUT(
                    predFx,
                    predFy,
                    baseX,
                    baseY
                );

                servoX = baseX;
                servoY = baseY;

                // ==========================
                // MOTOR CONTROL
                int errorY = fy - 240;

                if (abs(errorY) < 30)
                {
                    gpioWrite(MOTOR_IN1, 0);
                    gpioWrite(MOTOR_IN2, 0);
                }
                else if (errorY < 0)
                {
                    gpioWrite(MOTOR_IN1, 0);
                    gpioWrite(MOTOR_IN2, 1);
                }
                else
                {
                    gpioWrite(MOTOR_IN1, 1);
                    gpioWrite(MOTOR_IN2, 0);
                }

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

                // MOTOR STOP
                gpioWrite(MOTOR_IN1, 0);
                gpioWrite(MOTOR_IN2, 0);
            }

            // ==========================
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

        while (running)
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

    // ==========================
    // WAIT
    visionThread.join();
    servoThread.join();

    // ==========================
    // STOP EVERYTHING
    gpioServo(SERVO_X, 0);
    gpioServo(SERVO_Y, 0);

    gpioWrite(MOTOR_IN1, 0);
    gpioWrite(MOTOR_IN2, 0);

    gpioTerminate();

    return 0;
}