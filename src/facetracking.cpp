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
#include <string>
#include <stdexcept>

#define SERVO_X 17
#define SERVO_Y 27

constexpr int MOTOR_CENTER_DEADBAND_PX = 30;

enum class MotorDirection { Stop, Up, Down };

MotorDirection desiredMotorDirection(int faceY, int frameHeight)
{
    const int errorY = faceY - frameHeight / 2;
    if (errorY < -MOTOR_CENTER_DEADBAND_PX)
        return MotorDirection::Up;
    if (errorY > MOTOR_CENTER_DEADBAND_PX)
        return MotorDirection::Down;
    return MotorDirection::Stop;
}

#ifndef CALIBRATION_FILE
#define CALIBRATION_FILE "data/fixed_to_servo.txt"
#endif
#ifndef CASCADE_FILE
#define CASCADE_FILE "data/haarcascade_frontalface_default.xml"
#endif


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
        std::cout << "No fixed_to_servo.txt found\n";
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
int main(int argc, char *argv[])
{
    std::string fixedCameraDevice = "/dev/video0";
    try
    {
        if (argc >= 2)
        {
            const std::string requestedDevice = argv[1];
            if (!requestedDevice.empty() &&
                std::all_of(requestedDevice.begin(), requestedDevice.end(),
                            [](char c) { return c >= '0' && c <= '9'; }))
                fixedCameraDevice = "/dev/video" + requestedDevice;
            else
                fixedCameraDevice = requestedDevice;
        }
        if (argc > 2 || fixedCameraDevice.empty())
            throw std::invalid_argument("arguments");
    }
    catch (const std::exception &)
    {
        std::cerr << "Usage: facetracking [fixed_camera_device]\n";
        return 1;
    }

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

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);

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

                switch (desiredMotorDirection(fy, frame.rows))
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

            // ==========================
            // Preview the intended direction without energizing the relays.
            cv::line(frame, {0, frame.rows / 2},
                     {frame.cols - 1, frame.rows / 2}, {0, 255, 255}, 1);
            cv::putText(frame, "Motor preview: " + motorPreview + " (relay disabled)",
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

    gpioTerminate();

    return 0;
}
