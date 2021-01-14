#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt

def circle_IK(origin, radius, time, t, elbow):
    """
    Arguments:
        - origin (list): desired origin of the circle in the global reference frame
        - radius (float)
        - time (int/float): total time to complete the trajectory
        - t (int/float): instantaneous time value. increases incrementally
        - elbow (int): 1 for elbow up; -1 for elbow down
    """

    # 1. define the cartesian waypoints
    omega = (2*math.pi)/time # rad/s

    xe = origin[0] + radius * math.cos(omega*t)
    ye = origin[1] + radius * math.sin(omega*t)

    
    # 2. Inverse kinematics
    l1 = 0.25
    l2 = 0.25

    phi = math.atan2(ye, xe)
    phi += 2*math.pi if phi < 0 else False

    D = (xe**2 + ye**2 - l1**2 - l2**2)/(2*l1*l2)
    theta2 = elbow*math.atan2(math.sqrt(1-D**2), D)

    a = math.atan2(ye, xe)
    b = math.atan2(l2*math.sin(theta2), l1+l2*math.cos(theta2))
    
    theta1 = phi - math.atan2(l2*math.sin(theta2), l1+l2*math.cos(theta2))

    # if ye < 0:
    #     theta1 = theta1 + 2*math.pi

    q = [theta1, theta2]

    return q, [xe, ye], [a, b]
    # return q

if __name__ == "__main__":

    radius = 0.1
    time = 4
    r = 100
    origin = [0.2, 0.2]
    elbow = 1
    t = np.linspace(0, 2*time, num=r)


    q1 = []
    q2 = []
    xe = []
    ye = []
    a = []
    b = []

    for i in range(r):
        q, X, A = circle_IK(origin, radius, time, t[i], elbow)
        # q = circle_IK(radius, time, t[i])
        # q.append(circle_IK(radius, time, t[i]))

        q1.append(q[0])
        q2.append(q[1])

        xe.append(X[0])
        ye.append(X[1])

        a.append(A[0])
        b.append(A[1])
    
    # plt.plot(t, a, marker='.' )
    plt.plot(t, q1, marker='o' )
    plt.plot(t, q2, marker='.' )
    # plt.plot(t, b)
    # plt.plot(xe, ye)
    plt.show()

    # n = 100
    # dt = 0.01
    # radius = 0.45
    # duration = 3
    # origin = [0, 0]

    # q, X, A = circle_IK(origin, radius, duration, 1)

    # print(q)
