import numpy as np

data = np.genfromtxt("./robot_traj.csv", delimiter=",", dtype=float)

print(data.shape)
print(np.max(data[:, 0:2]))
print(np.min(data[:, 0:2]))

resize_data = 0.6 * data
# offset = 2900
# subsample_data = resize_data[offset:, :]
# save_data = np.append(resize_data[:offset, :], subsample_data[::5, :], axis=0)

# print(save_data.shape)
np.savetxt("./robot_traj_resize.csv", resize_data, delimiter=",")