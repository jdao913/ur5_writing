# Build for linux by default
PLATFORM := LINUX

CC     := g++

# Platform-specific settings
ifeq ($(PLATFORM), WIN)
# CC     := x86_64-w64-mingw32-gcc	# gcc 
MJ_PATH	:= C:/mujoco200_win64
else

MJ_PATH	:= $(HOME)/.mujoco/mujoco200_linux
endif

# Compilation settings
INC     := -Iinclude -I$(MJ_PATH)/include
CFLAGS  := -std=c++11 -Wall -Wextra -O3 -flto -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS   := -L$(MJ_PATH)/bin -lmujoco200 -lGL -lglew $(MJ_PATH)/bin/libglfw.so.3

all: sim control

# Normal targets
clean:
	rm -f ./sim

sim: 
	$(CC) $(INC) $(CFLAGS) sim.cpp mj_robot.cpp $(LIBS) -o sim

control:
	$(CC) $(INC) $(CFLAGS) control.cpp mj_robot.cpp $(LIBS) -o control


# Virtual targets
.PHONY: all sim control clean