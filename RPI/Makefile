# Makefile for RPI Multithread Communication Center

# Instructions:
# 1. To build the control center executable, run: make ctrl_center
# 2. To build the Android simulation executable, run: make and_center
# 3. To build the STM32 simulation executable, run: make st_center
# 4. To build the test center executable, run: make test_center
# 5. To enable livestream-based Task 2 detection, add USE_LIVESTREAM_TASK2=1 to the make command, e.g.:
#    make ctrl_center USE_LIVESTREAM_TASK2=1


CC = gcc
CFLAGS = -Wall -g
LIBS = -lpthread -lcurl
SRC = multithread_communication.c json_parser.c rpi_hal.c

# Optional build-time toggles
# Example:
# make ctrl_center USE_LIVESTREAM_TASK2=1
USE_LIVESTREAM_TASK2 ?= 0
CFLAGS += -DUSE_LIVESTREAM_TASK2=$(USE_LIVESTREAM_TASK2)

# Add suffix when livestream mode is enabled
ifeq ($(USE_LIVESTREAM_TASK2),1)
SUFFIX = _task2LS
else
SUFFIX =
endif

CTRL_OUT = ctrl_center$(SUFFIX)
AND_OUT  = and_center$(SUFFIX)
STM_OUT  = st_center$(SUFFIX)
TEST_OUT = test_center$(SUFFIX)

.PHONY: ctrl_center and_center st_center test_center clean

ctrl_center: $(SRC)
	$(CC) $(CFLAGS) $(SRC) -o $(CTRL_OUT) $(LIBS)

and_center: $(SRC)
	$(CC) $(CFLAGS) -DANDROID_SIM=1 $(SRC) -o $(AND_OUT) $(LIBS)

st_center: $(SRC)
	$(CC) $(CFLAGS) -DSTM32_SIM=1 $(SRC) -o $(STM_OUT) $(LIBS)

test_center: $(SRC)
	$(CC) $(CFLAGS) -DSTM32_SIM=1 -DANDROID_SIM=1 -DRPI_TESTING $(SRC) -o $(TEST_OUT) $(LIBS)

clean:
	rm -f ctrl_center ctrl_center_task2LS \
	      and_center and_center_task2LS \
	      st_center st_center_task2LS \
	      test_center test_center_task2LS