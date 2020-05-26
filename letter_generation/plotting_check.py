import matplotlib.pyplot as plt
import csv
import numpy as np

x = []
y = []

# with open('your_filename.csv') as csvfile:
with open('a_cap.csv') as csvfile:
    plots = csv.reader(csvfile,delimiter=',')
    for row in plots:
        x.append(float(row[0]))
        y.append(float(row[1]))


plt.plot(x,y,marker='o')

plt.show()
