"""
@author: Nima Lotfi, Southern Illinois University Edwardsville
@copyright: Copyright (C) 2021 Nima Lotfi
@license: This file is part of the scripts included in the two-part paper, titled # "Use of Open-source Software Platforms in Mechatronics and Robotics Engineering Education"

"""

import numpy as np
from scipy.integrate import solve_ivp
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
from IPython import get_ipython

get_ipython().magic('reset -sf')

def model(t, x, u):
    
    theta, omega, i = x
    dxdt = [omega, -(b/J)*omega+(Kt/J)*i, -(Kb/L)*omega-(R/L)*i+(1/L)*u]
    
    return dxdt

# def model(t, x, u):

#     A = [[0, 1, 0],[0, -(b/J), (Kt/J)],[0, -(Kb/L), -(R/L)]]
#     B = [[0], [0], [1/L]]
#     dxdt = np.dot(A,x)+ np.dot(B,u)
  
#     return dxdt

# model parameters
J = 0.0026
b = 0.01
Kt = 0.66
Kb = 0.66
R = 2.62
L = 0.05

# input voltage magnitude
v = 24

dt = 0.001
t0 = 0
tf = 0.5
t = np.arange(t0, tf, dt)

x0 = [0, 0, 0]

sol = solve_ivp(model, (t0, tf), x0, dense_output= True, vectorized= True, args = (v, ))
x = sol.sol(t)

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(8, 6), tight_layout=True, sharex=True)
ax1.plot(t,x[0, :])
ax1.grid(True)
ax1.set_ylabel(r'$\theta(t)$ [rad]', fontsize = 12)
ax1.tick_params(labelsize=12)
ax2.plot(t,x[1, :])
ax2.set_ylabel(r'$\omega(t)$ [rad/s]', fontsize = 12)
ax2.tick_params(labelsize=12)
ax2.grid(True)
ax3.plot(t,x[2, :])
ax3.set_ylabel(r'$i(t)$ [A]', fontsize = 12)
ax3.set_xlabel(r'Time [s]', fontsize=12)
ax3.tick_params(labelsize=12)
ax3.grid(True)
fig.show()

