cat << 'EOF' > README.md
# FaceTracker SDK for RPi4

1. Run setup:
   chmod +x setup_project.sh
   ./setup_project.sh

2. Compile:
   g++ -o facetracking facetracking.cpp `pkg-config --cflags --libs opencv4` -lpigpio -lpthread -lrt
   g++ -o calibration calibration.cpp `pkg-config --cflags --libs opencv4` -lpigpio -lpthread -lrt

4. Run:
   sudo ./FaceTracker
EOF# FaceTracker
# FaceTracker
# FaceTracker
# FaceTracker
