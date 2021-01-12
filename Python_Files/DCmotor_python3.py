import numpy as np
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
import control.matlab as matlab
from IPython import get_ipython

get_ipython().magic('reset -sf')

# model parameters
J = 0.0026
b = 0.01
Kt = 0.66
Kb = 0.66
R = 2.62
L = 0.05

# A = [[0, 1, 0],[0, -(b/J), (Kt/J)],[0, -(Kb/L), -(R/L)]]
# B = [[0], [0], [1/L]]
# C = [[1, 0, 0],[0, 1, 0],[0, 0, 1]]
A = np.array([0,1,0,0,-(b/J),(Kt/J),0,-(Kb/L),-(R/L)]).reshape(3,3)
B = np.array([0,0,1/L])[:,None]
C = np.identity(3)
D = 0

ssModel = matlab.ss(A,B,C,D)

dt = 0.001
t_init = 0
t_final = 0.5
t = np.arange(t_init,t_final,dt)
v = 24

x, t = matlab.step(v*ssModel, t)


fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), tight_layout=True, sharex=True)
ax1.plot(t,x[:, 0])
ax1.grid(True)
ax1.set_ylabel(r'$\theta(t)$ [rad]', fontsize = 12)
ax1.tick_params(labelsize=12)
ax2.plot(t,x[:, 1])
ax2.set_ylabel(r'$\omega(t)$ [rad/s]', fontsize = 12)
ax2.tick_params(labelsize=12)
ax2.grid(True)
ax3.plot(t,x[:, 2])
ax3.set_ylabel(r'$i(t)$ [A]', fontsize = 12)
ax3.set_xlabel(r'Time [s]', fontsize=12)
ax3.tick_params(labelsize=12)
ax3.grid(True)
fig.show()