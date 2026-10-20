#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include "facetracker/sdk.hpp"

#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <string>
#include <stdexcept>

#define SERVO_X 17
#define SERVO_Y 27

#ifndef CALIBRATION_FILE
#define CALIBRATION_FILE "data/fixed_to_servo.txt"
#endif
// ==========================
// CALIBRATION STRUCT
using CalibrationPoint = facetracker::CalibrationPoint;

std::vector<CalibrationPoint> LUT;

std::string cameraPath(const std::string &device)
{
    if (!device.empty() &&
        std::all_of(device.begin(), device.end(),
                    [](char c) { return c >= '0' && c <= '9'; }))
        return "/dev/video" + device;
    return device;
}

// ==========================
int main(int argc, char *argv[])
{
    std::string fixedDevice = "/dev/video0";
    std::string movingDevice = "/dev/video2";
    int cornersX = 11;
    int cornersY = 7;
    try
    {
        if (argc == 5)
        {
            fixedDevice = cameraPath(argv[1]);
            movingDevice = cameraPath(argv[2]);
            cornersX = std::stoi(argv[3]);
            cornersY = std::stoi(argv[4]);
        }
        else if (argc != 1)
        {
            throw std::invalid_argument("argument count");
        }
    }
    catch (const std::exception &)
    {
        std::cerr << "Usage: calibration [fixed_camera_device moving_camera_device "
                     "inner_corners_x inner_corners_y]\n";
        return 1;
    }
    if (fixedDevice.empty() || movingDevice.empty() ||
        fixedDevice == movingDevice || cornersX < 3 || cornersY < 3 ||
        cornersX > 20 || cornersY > 20)
    {
        std::cerr << "Invalid camera devices or chessboard size\n";
        return 1;
    }
    const cv::Size boardSize(cornersX, cornersY);

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
    cv::VideoCapture cap;
    cv::VideoCapture movingCap;
    if (!cap.open(fixedDevice, cv::CAP_V4L2))
    {
        std::cerr << "Cannot open fixed camera: " << fixedDevice << "\n";
        gpioTerminate();
        return 1;
    }
    if (!movingCap.open(movingDevice, cv::CAP_V4L2))
    {
        std::cerr << "Cannot open moving camera: " << movingDevice << "\n";
        std::cerr << "Check /dev/video* and v4l2-ctl --list-devices. "
                     "The second camera is not always /dev/video2.\n";
        gpioTerminate();
        return 1;
    }

    cap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    cap.set(cv::CAP_PROP_FPS, 30);
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // This camera supports MJPEG at 640x480/15 fps on the Raspberry Pi.
    // Request it before the resolution to avoid a bandwidth-heavy raw stream.
    movingCap.set(cv::CAP_PROP_FOURCC,
                  cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    movingCap.set(cv::CAP_PROP_FRAME_WIDTH, 640);
    movingCap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
    movingCap.set(cv::CAP_PROP_FPS, 15);
    movingCap.set(cv::CAP_PROP_BUFFERSIZE, 1);

    // ==========================
    // START POSITIONS
    int servoX = 90;
    int servoY = 90;

    // ==========================
    // SAFE CENTER
    gpioServo(SERVO_X, 1500);
    gpioServo(SERVO_Y, 1500);

    std::cout << "\n=== FIXED CAMERA TO SERVO CALIBRATION ===\n";
    std::cout << "Chessboard: " << cornersX << "x" << cornersY
              << " inner corners\n";
    std::cout << "A/D -> X servo\n";
    std::cout << "W/S -> Y servo\n";
    std::cout << "Aim until the board is centered in the MOVING camera\n";
    std::cout << "SPACE -> Record fixed-camera board position and servo angles\n";
    std::cout << "P -> Save all points to fixed_to_servo.txt\n";
    std::cout << "C -> Clear all LUT points\n";
    std::cout << "Q -> Quit\n\n";

    // ==========================
    // BOARD STABILITY
    int stableCount = 0;
    int movingStableCount = 0;
    cv::Point2f lastCenter(-1.0f, -1.0f);
    cv::Point2f lastMovingCenter(-1.0f, -1.0f);
    int failedReads = 0;
    bool cameraError = false;

    while (true)
    {
        cv::Mat frame;
        cv::Mat movingFrame;

        if (!cap.grab() || !movingCap.grab() ||
            !cap.retrieve(frame) || !movingCap.retrieve(movingFrame))
        {
            if (++failedReads >= 30)
            {
                std::cerr << "Camera frame read failed repeatedly\n";
                cameraError = true;
                break;
            }
            continue;
        }

        if (frame.empty() || movingFrame.empty())
            continue;
        failedReads = 0;

        if (frame.cols != 640 || frame.rows != 480)
        {
            std::cerr << "Fixed camera must deliver 640x480 frames\n";
            cameraError = true;
            break;
        }

        // ==========================
        // GRAY
        cv::Mat gray;
        cv::Mat movingGray;

        cv::cvtColor(
            frame,
            gray,
            cv::COLOR_BGR2GRAY
        );
        cv::cvtColor(movingFrame, movingGray, cv::COLOR_BGR2GRAY);

        std::vector<cv::Point2f> corners;
        std::vector<cv::Point2f> movingCorners;
        const bool found = cv::findChessboardCorners(
            gray, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK
        );
        const bool movingFound = cv::findChessboardCorners(
            movingGray, boardSize, movingCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK
        );

        int px = -1;
        int py = -1;
        bool aimed = false;

        // ==========================
        // BOARD FOUND IN THE FIXED CAMERA
        if (found)
        {
            cv::cornerSubPix(
                gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS |
                                 cv::TermCriteria::COUNT, 30, 0.1)
            );
            cv::drawChessboardCorners(frame, boardSize, corners, true);

            cv::Point2f center(0.0f, 0.0f);
            for (const auto &corner : corners)
            {
                center += corner;
            }
            center *= 1.0f / static_cast<float>(corners.size());

            if (lastCenter.x >= 0.0f &&
                cv::norm(center - lastCenter) < 5.0)
                ++stableCount;
            else
                stableCount = 0;
            lastCenter = center;

            if (stableCount >= 3)
            {
                px = cvRound(center.x);
                py = cvRound(center.y);
                cv::circle(frame, {px, py}, 5, {0, 255, 0}, -1);
            }
        }
        else
        {
            stableCount = 0;
            lastCenter = {-1.0f, -1.0f};
        }

        if (movingFound)
        {
            cv::cornerSubPix(
                movingGray, movingCorners, cv::Size(11, 11),
                cv::Size(-1, -1),
                cv::TermCriteria(cv::TermCriteria::EPS |
                                 cv::TermCriteria::COUNT, 30, 0.1)
            );
            cv::drawChessboardCorners(movingFrame, boardSize,
                                      movingCorners, true);
            cv::Point2f movingCenter(0.0f, 0.0f);
            for (const auto &corner : movingCorners)
                movingCenter += corner;
            movingCenter *= 1.0f / static_cast<float>(movingCorners.size());
            if (lastMovingCenter.x >= 0.0f &&
                cv::norm(movingCenter - lastMovingCenter) < 5.0)
                ++movingStableCount;
            else
                movingStableCount = 0;
            aimed = movingStableCount >= 3 &&
                    cv::norm(movingCenter -
                             cv::Point2f(movingFrame.cols / 2.0f,
                                         movingFrame.rows / 2.0f)) < 20.0;
            cv::circle(movingFrame, movingCenter, 6,
                       aimed ? cv::Scalar(0, 255, 0)
                             : cv::Scalar(0, 0, 255), -1);
            lastMovingCenter = movingCenter;
        }
        else
        {
            movingStableCount = 0;
            lastMovingCenter = {-1.0f, -1.0f};
        }

        cv::drawMarker(movingFrame,
                       {movingFrame.cols / 2, movingFrame.rows / 2},
                       {0, 255, 255}, cv::MARKER_CROSS, 24, 2);

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

        cv::putText(frame,
                    px >= 0 && aimed ? "Ready - SPACE to record"
                                     : "Find board in both cameras and center moving view",
                    {20, 120}, cv::FONT_HERSHEY_SIMPLEX, 0.45,
                    px >= 0 && aimed ? cv::Scalar(0, 255, 0)
                                     : cv::Scalar(0, 200, 255), 1);

        // ==========================
        // SHOW WINDOW
        cv::imshow(
            "Fixed camera",
            frame
        );
        cv::imshow("Moving camera", movingFrame);

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
            px != -1 &&
            py != -1 && aimed)
        {
            bool tooClose = false;

            for (auto &p : LUT)
            {
                if (
                    abs(p.px - px) < 20 &&
                    abs(p.py - py) < 20
                )
                {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose)
            {
                LUT.push_back({
                    px,
                    py,
                    servoX,
                    servoY
                });

                std::cout
                    << "Saved: "
                    << px << "," << py
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
        else if (key == ' ')
        {
            std::cout << "Board must be stable in fixed view and centered "
                         "in moving view\n";
        }

        // ==========================
        // APPEND SAVE FILE
        if (key == 'p')
        {
            std::ofstream file(
                CALIBRATION_FILE
            );

            if (!file)
            {
                std::cerr << "Could not open " << CALIBRATION_FILE
                          << " for writing\n";
                continue;
            }

            for (auto &p : LUT)
            {
                file
                    << p.px << " "
                    << p.py << " "
                    << p.sx << " "
                    << p.sy << "\n";
            }

            file.flush();
            if (!file)
            {
                std::cerr << "Could not save " << CALIBRATION_FILE << "\n";
                continue;
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

    return cameraError ? 1 : 0;
}
