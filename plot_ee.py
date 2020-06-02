import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt("./robot_test_inv_dyn.csv", delimiter=",", dtype=float, skip_footer=1)
true_data = np.genfromtxt("./trajectory_gen/robot_traj_resize.csv", delimiter=",", dtype=float)
# print(true_data.shape)

fig, ax = plt.subplots()
ax.plot(data[:, 0], data[:, 1], label="actual ee")
# offset = 2900
ax.plot(true_data[:, 0], true_data[:, 1], label="desired ee")
ax.legend()
plt.show()
