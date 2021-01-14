#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time
from std_msgs.msg import Float64
from circle_test import circle_IK
from controller_manager_msgs.srv import SwitchController

def draw_circle():

    # Initialize node
    rospy.init_node('draw_circle_node')

    # while rospy.get_rostime().to_sec() == 0.0:
    #     time.sleep(0.1)
    #     print(rospy.get_rostime().to_sec())
    
    # Initiate publisher
    trajectory_pub = rospy.Publisher('/planar_robot/joint_trajectory_controller/command',
             JointTrajectory, queue_size=5)
    
    joint_1_pub = rospy.Publisher('/planar_robot/joint_1_position_controller/command',
             Float64, queue_size=1)

    joint_2_pub = rospy.Publisher('/planar_robot/joint_2_position_controller/command',
             Float64, queue_size=1)


    # Set up the service
    rospy.wait_for_service('/planar_robot/controller_manager/switch_controller')


    # Ensure publisher is connected
    rate = rospy.Rate(1)
    while trajectory_pub.get_num_connections() == 0 or joint_1_pub.get_num_connections() == 0 or joint_2_pub.get_num_connections() == 0:
        rate.sleep()
    

    # Create instances of Float64 for joint command
    j1 = Float64()
    j2 = Float64()

    # Define circle parameters
    n = 100
    dt = 0.01
    radius = 0.1
    circle_period = 4
    duration = 6
    elbow = 1 # elbow down
    origin = [0.2, 0.2]

    # Calculate first point on the circle
    q, X, A = circle_IK(origin, radius, circle_period, 0, elbow)
    j1.data = q[0]
    j2.data = q[1]

    
    # Publish the commands

    # time.sleep(5)

    rospy.loginfo("<<< STATUS: Moving to Start Position >>>")

    joint_1_pub.publish(j1)
    joint_2_pub.publish(j2)

    time.sleep(4)

    # Switch controller
    try:
        switch_controller = rospy.ServiceProxy(
                            '/planar_robot/controller_manager/switch_controller', SwitchController)
        switch = SwitchController()
        switch.start_controllers = ['joint_trajectory_controller']
        switch.stop_controllers = ['joint_1_position_controller', 'joint_2_position_controller']
        switch.strictness = 2
        

        switch_controller.call(start_controllers=['joint_trajectory_controller'],
                            stop_controllers=['joint_1_position_controller', 'joint_2_position_controller'],
                            strictness=2)

    except rospy.ServiceException, e:
      print "Service call failed: %s"%e

    

    # Create an instance of type JointTrajectory
    traj = JointTrajectory()

    # Set "traj" header
    traj.header.stamp = rospy.Time.now()
    traj.header.frame_id = "base_link"

    # Set "traj" joint names
    traj.joint_names.append("joint_1")
    traj.joint_names.append("joint_2")

    # Create joint trajectory for circle motion
    for i in range(n):

        # Call the circle_IK function
        q, X, A = circle_IK(origin, radius, circle_period, i*dt*duration, elbow)
        
        point = JointTrajectoryPoint()
        point.positions.append(q[0])
        point.positions.append(q[1])

        traj.points.append(point)

        traj.points[i].time_from_start = rospy.Duration.from_sec(i*dt*duration)
        # rospy.loginfo("Status: joint_angles[%f][%f, %f]", i*dt*duration, q[0], q[1])
    
    rospy.loginfo("<<< STATUS: Circle Drawing Started >>>")

    trajectory_pub.publish(traj)
    
    time.sleep(6)

    rospy.loginfo("<<< STATUS: Circle Drawing Complete >>>")

if __name__ == "__main__":
    try:
        draw_circle()
    except rospy.ROSInterruptException:
        pass