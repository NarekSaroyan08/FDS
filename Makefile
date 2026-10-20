# ===========================================
# FaceTracker build (Raspberry Pi 4)
# ===========================================

CXX      ?= g++
CXXFLAGS ?= -O2 -Wall -std=c++17
CPPFLAGS += -Iinclude
OPENCV_CFLAGS = $(shell pkg-config --cflags opencv4)
OPENCV_LIBS   = $(shell pkg-config --libs opencv4)
LDLIBS   += -lpigpio -lpthread -lrt

SRC_DIR   := src
BUILD_DIR := build

TARGETS := $(BUILD_DIR)/facetracking $(BUILD_DIR)/calibration
SDK_OBJ := $(BUILD_DIR)/sdk.o
SDK_LIB := $(BUILD_DIR)/libfacetracker_sdk.a
TEST_BIN := $(BUILD_DIR)/sdk_tests

.PHONY: all clean run calibrate test

all: $(TARGETS)

$(SDK_OBJ): $(SRC_DIR)/sdk.cpp include/facetracker/sdk.hpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -c -o $@ $<

$(SDK_LIB): $(SDK_OBJ)
	$(AR) rcs $@ $<

$(BUILD_DIR)/facetracking: $(SRC_DIR)/facetracking.cpp include/facetracker/sdk.hpp $(SDK_LIB) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $@ $< $(SDK_LIB) $(OPENCV_LIBS) $(LDLIBS)

$(BUILD_DIR)/calibration: $(SRC_DIR)/calibration.cpp include/facetracker/sdk.hpp | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) $(OPENCV_CFLAGS) -o $@ $< $(OPENCV_LIBS) $(LDLIBS)

$(TEST_BIN): tests/sdk_tests.cpp include/facetracker/sdk.hpp $(SDK_LIB) | $(BUILD_DIR)
	$(CXX) $(CPPFLAGS) $(CXXFLAGS) -o $@ $< $(SDK_LIB)

test: $(TEST_BIN)
	./$(TEST_BIN)

$(BUILD_DIR):
	mkdir -p $@

# Run from the project root so data/ paths resolve
run: $(BUILD_DIR)/facetracking
	sudo ./$(BUILD_DIR)/facetracking /dev/video0 --enable-motor

calibrate: $(BUILD_DIR)/calibration
	sudo ./$(BUILD_DIR)/calibration

clean:
	rm -f $(TARGETS) $(TEST_BIN) $(SDK_OBJ) $(SDK_LIB)
