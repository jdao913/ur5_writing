import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import get_letter_traj
from scipy.interpolate import interp1d

timestep = 0.01

def get_vel_accel(xd,yd,l):

    accel_sat = 9.81
    # forwrds difference
    xdotd_f = np.zeros((l))
    ydotd_f = np.zeros((l))

    xddotd_f = np.zeros((l))
    yddotd_f = np.zeros((l))

    for i in range(l-1):

        xdotd_f[i+1] = ((xd[i+1]-xd[i])/timestep)
        ydotd_f[i+1] = ((yd[i+1]-yd[i])/timestep)

    for i in range(l-1):
        xddotd_f[i+1] = ((xdotd_f[i+1]-xdotd_f[i])/timestep)
        # saturation line
        if xddotd_f[i] > accel_sat:
            xdotd_f[i] = 0
            xddotd_f[i] = 0
        elif xddotd_f[i] < -accel_sat:
            xdotd_f[i] = 0
            xddotd_f[i] = 0

        yddotd_f[i+1] = ((ydotd_f[i+1]-ydotd_f[i])/timestep)
        # staruation line
        if yddotd_f[i] > accel_sat:
            ydotd_f[i] = 0
            yddotd_f[i] = 0
        elif yddotd_f[i] < -accel_sat:
            ydotd_f[i] = 0
            yddotd_f[i] = 0

    # backwards difference
    xdotd_b = np.zeros((l))
    ydotd_b = np.zeros((l))

    xddotd_b = np.zeros((l))
    yddotd_b = np.zeros((l))

    for i in range(1,l):
        xdotd_b[i] = ((xd[i]-xd[i-1])/timestep)
        ydotd_b[i] = ((yd[i]-yd[i-1])/timestep)

    for i in range(1,l):
        xddotd_b[i] = ((xdotd_b[i]-xdotd_b[i-1])/timestep)
        # saturation line
        if xddotd_b[i] > accel_sat:
            xdotd_b[i] = 0
            xddotd_b[i] = 0
        elif xddotd_b[i] < -accel_sat:
            xdotd_b[i] = 0
            xddotd_b[i] = 0

        yddotd_b[i] = ((ydotd_b[i]-ydotd_b[i-1])/timestep)
        # saturation line
        if yddotd_b[i] > accel_sat:
            ydotd_b[i] = 0
            yddotd_b[i] = 0
        elif yddotd_b[i] < -accel_sat:
            ydotd_b[i] = 0
            yddotd_b[i] = 0

    # average the forward and backwards
    xdotd = (xdotd_f + xdotd_b)/2
    ydotd = (ydotd_f + ydotd_b)/2

    xddotd = (xddotd_f + xddotd_b)/2
    yddotd = (yddotd_f + yddotd_b)/2

    return xdotd, ydotd, xddotd, yddotd

def traj_gen(input_str):
    num_letters = len(input_str)

    for num in range(num_letters):
        letter = input_str[num]

        if num == 0:
            # first letter
            xd, yd, l = get_letter_traj.get_traj(letter)

            to_append = np.zeros((l,6))
            to_append[:,0] = xd
            to_append[:,1] = yd

            trajectory = to_append
            x_offset = xd[-1]
            y_offset = min(yd)

            penum_value_x = xd[-2]
            penum_value_y = yd[-2]
            last_value_x = x_offset
            last_value_y = yd[-1]

        else:
            xd, yd, l = get_letter_traj.get_traj(letter)

            xd = [x + x_offset + 0.5 for x in xd]
            yd = [y + y_offset for y in yd]

            to_append = np.zeros((l,6))
            to_append[:,0] = xd
            to_append[:,1] = yd

            trajectory = np.append(trajectory,to_append,axis=0)

            x_offset = xd[-1]
            last_value_x = x_offset
            last_value_y = yd[-1]

    # can't do this bc multiple y for one x
    l = trajectory.shape[0]
    #trajectory =np.array([[4,1,0,0,0,4],[2,2,0,2,0,0],[1,3,1,0,0,0],[3,4,0,0,3,0]])
    trajectory_sort = trajectory[np.argsort(trajectory[:,0])]
    #print(trajectory)
    #print(trajectory_sort)
    np.savetxt("traj_sort.csv", trajectory_sort, delimiter=",")
    xdnew = np.linspace(trajectory_sort[0,0],trajectory_sort[l-1,0],num=l*3, endpoint = True)

    x = np.array(trajectory_sort[:,0])
    y = np.array(trajectory_sort[:,1])
    #print(x.shape)
    get_ydnew = interp1d(x,y,kind='cubic')

    ydnew = get_ydnew(xdnew)
    #print(ydnew.shape)
    xdotd, ydotd, xddotd, yddotd = get_vel_accel(list(xdnew),list(xdnew),l*3)
    trajectory_sort[:,2] = xdotd
    trajectory_sort[:,3] = ydotd
    trajectory_sort[:,4] = xddotd
    trajectory_sort[:,5] = yddotd

    print(trajectory.shape)
    print('hi')


    return trajectory

trajectory = traj_gen('Hello')

fig, ax = plt.subplots(3, 1)

ax[0].plot(trajectory[:,0],trajectory[:,1])
#ax.plot(endeff_position[:,1])
#ax.legend()
ax[0].set(xlabel='X Position', ylabel='Y Position')

ax[1].plot(trajectory[:,2], label='xvel')
ax[1].plot(trajectory[:,3], label='yvel')
ax[1].legend()

ax[2].plot(trajectory[:,4], label='xaccel')
ax[2].plot(trajectory[:,5], label='yaccel')
ax[2].legend()

plt.show()
