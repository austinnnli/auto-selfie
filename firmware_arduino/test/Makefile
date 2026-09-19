# Compile the Arduino sketch on the host, without the Arduino IDE.
#
#     make -C firmware_arduino/test
#
# This does NOT run anything - it proves the sketch COMPILES, which is the
# check worth having before walking to the bench.  The stubs in stubs/ stand
# in for the Arduino-ESP32 core; they are declarations only.
#
# Both core versions are checked, because ledcAttach/ledcWrite changed
# signature between Arduino-ESP32 2.x and 3.x and a board set up for either
# one should build.

CC      ?= gcc
CXX     ?= g++
SKETCH   = ../framing_rover
INCLUDES = -Istubs -I$(SKETCH)
CFLAGS   = -std=gnu11 -Wall -Wextra -Werror -Wno-unused-parameter -O1
CXXFLAGS = -std=gnu++17 -Wall -Wextra -Werror -Wno-unused-parameter -O1

C_SRCS   = $(SKETCH)/drive.c $(SKETCH)/tilt.c $(SKETCH)/safety.c $(SKETCH)/tasks.c
CPP_SRCS = $(SKETCH)/hal_arduino.cpp $(SKETCH)/net_arduino.cpp

.PHONY: all core3 core2 clean
all: core3 core2
	@echo "Arduino sketch compiles for core 2.x and 3.x"

core3:
	@echo "--- Arduino-ESP32 core 3.x ---"
	@$(CC)  $(CFLAGS)   $(INCLUDES) -DESP_ARDUINO_VERSION_MAJOR=3 -fsyntax-only $(C_SRCS)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -DESP_ARDUINO_VERSION_MAJOR=3 -fsyntax-only $(CPP_SRCS)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -DESP_ARDUINO_VERSION_MAJOR=3 -fsyntax-only \
		-x c++ $(SKETCH)/framing_rover.ino

core2:
	@echo "--- Arduino-ESP32 core 2.x ---"
	@$(CC)  $(CFLAGS)   $(INCLUDES) -DESP_ARDUINO_VERSION_MAJOR=2 -fsyntax-only $(C_SRCS)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -DESP_ARDUINO_VERSION_MAJOR=2 -fsyntax-only $(CPP_SRCS)
	@$(CXX) $(CXXFLAGS) $(INCLUDES) -DESP_ARDUINO_VERSION_MAJOR=2 -fsyntax-only \
		-x c++ $(SKETCH)/framing_rover.ino

clean:
	@rm -f *.o
