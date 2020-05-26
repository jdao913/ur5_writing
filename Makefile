
# Compilation settings
INC     := -Iinclude -I$(HOME)/.mujoco/mujoco200_linux/include
CFLAGS  := -std=c++11 -Wall -Wextra -O3 -flto -mavx -pthread -Wl,-rpath,'$$ORIGIN'
LIBS   := -L$(HOME)/.mujoco/mujoco200_linux/bin -lmujoco200 -lGL -lglew ~/.mujoco/mujoco200_linux/bin/libglfw.so.3

all: sim

# Normal targets
clean:
	rm -f ./sim

sim: 
	g++ $(INC) $(CFLAGS) sim.cpp ur5.cpp $(LIBS) -o sim

# Virtual targets
.PHONY: all sim clean