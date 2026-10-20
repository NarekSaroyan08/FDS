# ===========================================
# FaceTracker build (Raspberry Pi 4)
# ===========================================

CXX      ?= g++
CXXFLAGS ?= -O2 -Wall -std=c++17
CXXFLAGS += $(shell pkg-config --cflags opencv4)
LDLIBS   += $(shell pkg-config --libs opencv4) -lpigpio -lpthread -lrt

SRC_DIR   := src
BUILD_DIR := build

TARGETS := $(BUILD_DIR)/facetracking $(BUILD_DIR)/calibration

.PHONY: all clean run run-motor calibrate

all: $(TARGETS)

$(BUILD_DIR)/%: $(SRC_DIR)/%.cpp | $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) -o $@ $< $(LDLIBS)

$(BUILD_DIR):
	mkdir -p $@

# Run from the project root so data/ paths resolve
run: $(BUILD_DIR)/facetracking
	sudo ./$(BUILD_DIR)/facetracking

run-motor: $(BUILD_DIR)/facetracking
	sudo ./$(BUILD_DIR)/facetracking /dev/video0 --enable-motor

calibrate: $(BUILD_DIR)/calibration
	sudo ./$(BUILD_DIR)/calibration

clean:
	rm -f $(TARGETS)
