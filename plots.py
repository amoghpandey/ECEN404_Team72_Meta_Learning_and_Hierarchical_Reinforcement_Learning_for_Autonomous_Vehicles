import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

df = pd.read_csv('poses.csv')

x = df['x']
y = df['y']


plt.plot(x,y, label = 'traj')
plt.grid()
plt.title('Trajectory')
plt.xlabel('y')
plt.ylabel('x')
plt.xlim(-12,-6)
plt.ylim(-10, -5)
plt.plot(-10,-7, 'ro', label = 'start')
plt.plot(-8,-8, 'go', label = 'goal')
plt.legend(loc = 'upper right')
plt.show()
