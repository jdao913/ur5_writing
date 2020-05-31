import numpy as np
import matplotlib.pyplot as plt

data = np.genfromtxt("./letter_test.csv", delimiter=",", dtype=float)
true_data = np.genfromtxt("./trajectory_gen/cap_a_traj.csv", delimiter=",", dtype=float)


plt.plot(data[:, 0], data[:, 1])
plt.plot(true_data[:, 0], true_data[:, 1])
plt.show()
