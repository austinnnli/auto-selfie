# Host tests for the firmware logic.  No ESP-IDF, no hardware.
#
#     make -C firmware/test          build and run
#     make -C firmware/test clean
#
# drive.c, tilt.c and safety.c are compiled against test/hal_stub.c instead of
# main/hal_esp32.c.  net.c and main.c are not built here: they are all ESP-IDF
# plumbing and nothing in them is worth a host test.

CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Werror -O1 -g -DROVER_HOST_TEST
INCLUDES = -I../main -I.
LDLIBS   = -lm

BIN_PROTO = test_protocol
BIN_LOGIC = test_drive_tilt

LOGIC_SRCS = test_drive_tilt.c hal_stub.c ../main/drive.c ../main/tilt.c ../main/safety.c

.PHONY: all run clean
all: run

$(BIN_PROTO): test_protocol.c ../main/protocol.h vectors.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ test_protocol.c $(LDLIBS)

$(BIN_LOGIC): $(LOGIC_SRCS) ../main/drive.h ../main/tilt.h ../main/safety.h ../main/rover_hal.h
	$(CC) $(CFLAGS) $(INCLUDES) -o $@ $(LOGIC_SRCS) $(LDLIBS)

run: $(BIN_PROTO) $(BIN_LOGIC)
	@./$(BIN_PROTO)
	@echo
	@./$(BIN_LOGIC)

clean:
	rm -f $(BIN_PROTO) $(BIN_LOGIC)
