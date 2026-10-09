#include <opencv2/opencv.hpp>
#include <pigpio.h>

#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>

#define SERVO_X 17
#define SERVO_Y 27

#ifndef CALIBRATION_FILE
#define CALIBRATION_FILE "data/calibration.txt"
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
int main()
{
    // ==========================
    // FIX OPENCV THREAD ISSUES
    cv::setNumThreads(1);

    // ==========================
    // INIT PIGPIO
    if (gpioInitialise() < 0)
    {
        std::cout << "pigpio init failed\n";
        return -1;
    }

    // ==========================
    // SAFE PWM FREQUENCY
    gpioSetPWMfrequency(SERVO_X, 50);
    gpioSetPWMfrequency(SERVO_Y, 50);

    // ==========================
    // LOAD OLD LUT DATA
    std::ifstream loadFile(CALIBRATION_FILE);

    if (loadFile.is_open())
    {
        CalibrationPoint p;

        while (loadFile >> p.px >> p.py >> p.sx >> p.sy)
        {
            LUT.push_back(p);
        }

        std::cout << "Loaded old LUT points: "
                  << LUT.size()
                  << std::endl;
    }

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
    // START POSITIONS
    int servoX = 90;
    int servoY = 90;

    // ==========================
    // SAFE CENTER
    gpioServo(SERVO_X, 1500);
    gpioServo(SERVO_Y, 1500);

    std::cout << "\n=== CALIBRATION MODE ===\n";
    std::cout << "A/D -> X servo\n";
    std::cout << "W/S -> Y servo\n";
    std::cout << "SPACE -> Save point\n";
    std::cout << "P -> Append save calibration.txt\n";
    std::cout << "C -> Clear all LUT points\n";
    std::cout << "Q -> Quit\n\n";

    // ==========================
    // FACE STABILITY
    int stableCount = 0;

    cv::Rect lastFace;

    while (true)
    {
        cv::Mat frame;

        if (!cap.read(frame))
            continue;

        if (frame.empty())
            continue;

        // ==========================
        // GRAY + SMALL
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

        // ==========================
        // DETECT
        std::vector<cv::Rect> faces;

        face_cascade.detectMultiScale(
            small,
            faces,
            1.2,
            3
        );

        int fx = -1;
        int fy = -1;

        // ==========================
        // FACE FOUND
        if (faces.size() > 0)
        {
            cv::Rect f = faces[0];

            // STABILITY CHECK
            if (
                abs(f.x - lastFace.x) < 10 &&
                abs(f.y - lastFace.y) < 10
            )
            {
                stableCount++;
            }
            else
            {
                stableCount = 0;
            }

            lastFace = f;

            // ONLY STABLE FACE
            if (stableCount > 3)
            {
                fx =
                    (f.x + f.width / 2) * 2;

                fy =
                    (f.y + f.height / 2) * 2;

                // DRAW FACE POINT
                cv::circle(
                    frame,
                    {fx, fy},
                    5,
                    {0,255,0},
                    -1
                );

                // DRAW RECTANGLE
                cv::rectangle(
                    frame,
                    cv::Rect(
                        f.x * 2,
                        f.y * 2,
                        f.width * 2,
                        f.height * 2
                    ),
                    {255,0,0},
                    2
                );
            }
        }

        // ==========================
        // GRID
        for (int x = 0; x < frame.cols; x += 80)
        {
            cv::line(
                frame,
                {x,0},
                {x,frame.rows},
                {50,50,50}
            );
        }

        for (int y = 0; y < frame.rows; y += 60)
        {
            cv::line(
                frame,
                {0,y},
                {frame.cols,y},
                {50,50,50}
            );
        }

        // ==========================
        // SAFE LIMITS
        servoX = std::clamp(
            servoX,
            20,
            160
        );

        servoY = std::clamp(
            servoY,
            20,
            160
        );

        // ==========================
        // SAFE SERVO PULSES
        int pulseX =
            1000 + servoX * 5;

        int pulseY =
            1000 + servoY * 5;

        gpioServo(
            SERVO_X,
            pulseX
        );

        gpioServo(
            SERVO_Y,
            pulseY
        );

        // ==========================
        // SHOW INFO
        cv::putText(
            frame,
            "ServoX: " + std::to_string(servoX),
            {20,30},
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            {0,255,0},
            2
        );

        cv::putText(
            frame,
            "ServoY: " + std::to_string(servoY),
            {20,60},
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            {0,255,0},
            2
        );

        cv::putText(
            frame,
            "Points: " + std::to_string(LUT.size()),
            {20,90},
            cv::FONT_HERSHEY_SIMPLEX,
            0.7,
            {0,255,0},
            2
        );

        // ==========================
        // SHOW WINDOW
        cv::imshow(
            "Calibration PRO",
            frame
        );

        // ==========================
        // KEYS
        char key = cv::waitKey(20);

        // X SERVO
        if (key == 'a')
            servoX -= 2;

        if (key == 'd')
            servoX += 2;

        // Y SERVO
        if (key == 'w')
            servoY -= 2;

        if (key == 's')
            servoY += 2;

        // ==========================
        // SAVE POINT
        if (key == ' ' &&
            fx != -1 &&
            fy != -1)
        {
            bool tooClose = false;

            for (auto &p : LUT)
            {
                if (
                    abs(p.px - fx) < 20 &&
                    abs(p.py - fy) < 20
                )
                {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose)
            {
                LUT.push_back({
                    fx,
                    fy,
                    servoX,
                    servoY
                });

                std::cout
                    << "Saved: "
                    << fx << "," << fy
                    << " -> "
                    << servoX << "," << servoY
                    << std::endl;
            }
            else
            {
                std::cout
                    << "Point too close to existing point\n";
            }
        }

        // ==========================
        // APPEND SAVE FILE
        if (key == 'p')
        {
            std::ofstream file(
                CALIBRATION_FILE
            );

            for (auto &p : LUT)
            {
                file
                    << p.px << " "
                    << p.py << " "
                    << p.sx << " "
                    << p.sy << "\n";
            }

            std::cout
                << "Saved ALL LUT points: "
                << LUT.size()
                << std::endl;
        }

        // ==========================
        // CLEAR LUT
        if (key == 'c')
        {
            LUT.clear();

            std::ofstream clearFile(
                CALIBRATION_FILE
            );

            clearFile.close();

            std::cout
                << "LUT cleared\n";
        }

        // ==========================
        // QUIT
        if (key == 'q')
            break;
    }

    // ==========================
    // STOP SERVOS
    gpioServo(SERVO_X, 0);
    gpioServo(SERVO_Y, 0);

    gpioTerminate();

    return 0;
}