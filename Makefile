COMMON=-O2 -I$(HOME)/.mujoco/mujoco200_linux/include -L$(HOME)/.mujoco/mujoco200_linux/bin -std=c++11 -mavx -pthread -Wl,-rpath,'$$ORIGIN'

all:
	g++ $(COMMON) sim.cpp      -lmujoco200 -lGL -lglew ~/.mujoco/mujoco200_linux/bin/libglfw.so.3 -o ./sim
	# rm *.o