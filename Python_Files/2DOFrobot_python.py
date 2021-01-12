# PID controller with a profiled reference trajectory generated within the loop

from IPython import get_ipython

get_ipython().magic('reset -sf')

import numpy as np
import matplotlib
matplotlib.rcParams['text.usetex'] = True
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp


def M_mat(th):
    
    m11 = I1 + I2 + m1*r1**2 + m2*(L1**2+r2**2)+2*m2*L1*r2*np.cos(th[1])
    m12 = I2 + m2*r2**2 + m2*L1*r2*np.cos(th[1])
    m21 = I2 + m2*r2**2 + m2*L1*r2*np.cos(th[1])
    m22 = I2 + m2*r2**2
    M = np.array([[m11, m12],[m21, m22]], dtype = 'float')
    
    return M

def C_mat(th, thdot):
    
    c11 = -m2*L1*r2*np.sin(th[1])*thdot[1]+b1
    c12 = -m2*L1*r2*np.sin(th[1])*(thdot[0]+thdot[1])
    c21 = m2*L1*r2*np.sin(th[1])*thdot[0]
    c22 = b2
    C = np.array([[c11, c12],[c21, c22]], dtype = 'float')
    
    return C

def g_mat(th):
    
    g1 = (m1*r1+m2*L1)*g*np.cos(th[0])+m2*r2*g*np.cos(th[0]+th[1])
    g2 = m2*r2*g*np.cos(th[0]+th[1])
    G = np.array([g1,g2], dtype = 'float')
    
    return G

def model(t, x, u):

    x1, x2, x3, x4 = x
    
    th = np.array([x1, x2], dtype = 'float')
    thdot = np.array([x3, x4], dtype = 'float')
    M_inv = np.linalg.inv(M_mat(th))
    thddot = np.dot(M_inv,(u-np.dot(C_mat(th, thdot),thdot)-g_mat(th)))
    
    dxdt = [x3, x4, thddot[0,:], thddot[1,:] ]
    
    return dxdt

def IK_fun(xe, ye):
    elbow = +1
    # elbow = -1
    theta = np.arctan2(ye, xe)
    if theta < 0:
        theta += 2*np.pi

    D = (xe**2+ye**2-L1**2-L2**2)/(2*L1*L2)
    th2 = np.arctan2(elbow*np.sqrt(1-D**2),D)
    th1 = theta-elbow*np.arctan2(L2*np.sin(th2),L1+L2*np.cos(th2))
    
    return th1, th2

def FK_fun(th1, th2):
    xe = L1*np.cos(th1) + L2*np.cos(th1+th2)
    ye = L1*np.sin(th1) + L2*np.sin(th1+th2)
    
    return xe, ye

def exp_dim(var):
    out = np.expand_dims(var, axis = 1)
    
    return out

# model parameters
L1 = 0.25
L2 = 0.25
r1 = L1/2
r2 = L2/2
m1 = 0.5
m2 = 0.5
g = 9.81
I1 = m1*L1**2/12
I2 = m1*L1**2/12
b1 = 1e-1
b2 = 1e-1

# simulation time
dt = 0.01
t_init = 0
t_final = 10
time = np.arange(t_init,t_final,dt)
th1 = 4
t_dwell = 1
th2 = th1 + t_dwell

#  initial conditions for the joint angles and the end-effector position
th_init = np.array([[np.pi/2],[0]], dtype = 'float')
thdot_init = np.array([[0],[0]], dtype = 'float')
state0 = np.concatenate((th_init,thdot_init))
state = state0
th = exp_dim(np.array([state[0,0], state[1,0]]))
thdot = exp_dim(np.array([state[2,0], state[3,0]]))
xe0, ye0 = FK_fun(th[0, 0], th[1, 0])
xe_act = [xe0]
ye_act = [ye0]

# the initial desired trajectory and the desired joint angles
def circ_traj(xo, yo, R, T, time):
    
    x = xo + R*np.cos((2/T)*np.pi*time)
    y = yo + R*np.sin((2/T)*np.pi*time) 

    return x, y        

xo = 0.1
yo = 0.2
R = 0.1
T = 4
xc0, yc0 = circ_traj(xo, yo, R, T, 0)

xe_d = [xe0]
ye_d = [ye0]
th1d, th2d = IK_fun(xe_d[0], ye_d[0])
thd = exp_dim(np.array([th1d, th2d]))


# controller parameters and initial control input
Kp = np.array([[20, 0],[0, 20]], dtype = 'float')
Ki = np.array([[40, 0],[0, 40]], dtype = 'float')
Kd = np.array([[2, 0],[0, 0.1]], dtype = 'float')


old_e = exp_dim(thd[:, 0]) - exp_dim(th[:, 0])
E = 0
tau = np.zeros((2,1))
tau_max = 10
tau_min = -10


for i in range(len(time)-1):
    
    # calculating the new system states by integrating the system dynamics
    t = time[i]
    tNext = t + dt
    state0 = np.squeeze(state[:,i]) # state0 should be 1D
    sol = solve_ivp(model, [t, tNext], state0, dense_output=True, vectorized=True, args=(exp_dim(tau[:, i]),))
    state_new = sol.sol(tNext)
    state = np.column_stack([state, state_new])
    th_new = exp_dim(np.array([state[0, i+1], state[1, i+1]]))
    th = np.column_stack([th, th_new])
    thdot_new = exp_dim(np.array([state[2, i+1], state[3, i+1]]))
    thdot = np.column_stack([thdot, thdot_new])
    
    # calcuating the end-effector position based on the calculated state variables
    xe_new, ye_new = FK_fun(th[0, i+1], th[1, i+1])
    xe_act += [xe_new] 
    ye_act += [ye_new]

    # calculating the desired joint angles based on the desired trajectory
    if time[i] < th1:
        xe_d += [(time[i]/th1)*(xc0-xe0)+xe0]
        ye_d += [(time[i]/th1)*(yc0-ye0)+ye0]
    elif time[i] >= th1 and time[i] < th2:
        xe_d += [xc0]
        ye_d += [yc0]        
    else:
        xc, yc = circ_traj(xo, yo, R, T, time[i]-th2)
        xe_d += [xc]
        ye_d += [yc]

    th1d, th2d = IK_fun(xe_d[i+1], ye_d[i+1])
    thd_new = exp_dim(np.array([th1d, th2d]))
    thd = np.column_stack([thd, thd_new])
    
    # calculating the control input based on the tracking error
    e = exp_dim(thd[:, i+1]) - exp_dim(th[:, i+1])
    E += e*dt
    edot = (e - old_e)/dt
    tau_new = np.dot(Kp,e) + np.dot(Kd,edot) + np.dot(Ki,E)
    tau_new [tau_new > 10] = tau_max
    tau_new [tau_new < -10] = tau_min
    tau = np.column_stack([tau, tau_new])
    old_e = e

    
plt.figure(1)
plt.clf()
plt.subplot(211)
plt.plot(time, thd[0, :], 'r:', label = 'Desired')
plt.plot(time, state[0, :], 'b-', label = 'Actual')
plt.ylabel(r'$\theta_1$ [rad]', fontsize = 12)
plt.grid()
plt.legend(loc = 'best', fontsize = 12)
plt.subplot(212)
plt.plot(time, thd[1, :], 'r:', label = 'Desired')
plt.plot(time, state[1, :], 'b-', label = 'Actual')
plt.ylabel(r'$\theta_2$ [rad]', fontsize = 12)
plt.xlabel('Time [s]', fontsize = 12)
plt.grid()
plt.legend(loc = 'best', fontsize = 12)
plt.show()

plt.figure(2)
plt.clf()
plt.plot(time,tau[0, :], 'b:', label = r'$u_1$')
plt.plot(time,tau[1, :], 'r--', label = r'$u_2$')
plt.legend(loc = 'best', fontsize = 12)
plt.ylabel('Control Input [N.m]', fontsize = 12)
plt.xlabel('Time [s]', fontsize = 12)
plt.grid()
plt.show()

plt.figure(3)
plt.clf()
plt.plot(xe_d, ye_d, 'r--', label = 'Desired')
plt.plot(xe_act, ye_act, 'b-', label = 'Actual')
plt.legend(loc = 'best', fontsize = 12)
plt.ylabel('y', fontsize = 12)
plt.xlabel('x', fontsize = 12)
plt.grid()
plt.show()