#!/bin/bash
# ===========================================
# Face Tracker Project Setup Script for RPi4
# ===========================================

echo "Starting Face Tracker SDK setup..."

sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential cmake pkg-config git
sudo apt install -y libjpeg-dev libtiff5-dev libpng-dev \
libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
libxvidcore-dev libx264-dev libgtk-3-dev libatlas-base-dev gfortran
sudo apt install -y pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
sudo apt install -y libopencv-dev python3-opencv

if [ ! -f "haarcascade_frontalface_default.xml" ]; then
    wget https://github.com/opencv/opencv/raw/master/data/haarcascades/haarcascade_frontalface_default.xml
fi

mkdir -p build
cd build

echo "Setup complete!"
echo "Compile using:"
echo "cd ~/FaceTracker_SDK"
echo "g++ -o FaceTracker FaceTracker.cpp \`pkg-config --cflags --libs opencv4\` -lpigpio -lpthread -lrt"
echo "Run with sudo ./FaceTracker"
