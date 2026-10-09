#!/bin/bash
# ===========================================
# Face Tracker Project Setup Script for RPi4
# ===========================================

set -e

# Always work from the project root
cd "$(dirname "$0")/.."

echo "Starting Face Tracker SDK setup..."

sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential cmake pkg-config git
sudo apt install -y libjpeg-dev libtiff5-dev libpng-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran
sudo apt install -y pigpio

# The programs link libpigpio directly (gpioInitialise), which fails
# while the pigpiod daemon holds the GPIO — keep the daemon off.
sudo systemctl disable --now pigpiod 2>/dev/null || true
sudo apt install -y libopencv-dev python3-opencv

mkdir -p data build

if [ ! -f "data/haarcascade_frontalface_default.xml" ]; then
    wget -O data/haarcascade_frontalface_default.xml \
        https://github.com/opencv/opencv/raw/master/data/haarcascades/haarcascade_frontalface_default.xml
fi

touch data/calibration.txt

echo "Setup complete!"
echo "Build:      make"
echo "Calibrate:  make calibrate   (or: sudo ./build/calibration)"
echo "Track:      make run         (or: sudo ./build/facetracking)"
