#!/usr/bin/env python

import rospy
import tf
from tf import transformations
from std_msgs.msg import Float64MultiArray
import time
from sensor_msgs.msg import JointState
import numpy as np


class DataRecorder():

    def __init__(self):
        # Initialize node
        rospy.init_node('data_recorder_node')
        # Set timer params
        self.rate = rospy.Rate(1)
        # Initiate publisher
        self.data_publisher = rospy.Publisher('/data_recorder', 
                            Float64MultiArray, queue_size=1)
        while self.data_publisher.get_num_connections() == 0:
            self.rate.sleep()
        # Initiate listener
        self.listener = tf.TransformListener()
        self.listener.waitForTransform('base_link', 'link_2', 
                                    rospy.Time(0), rospy.Duration(4))
        # Instantiate Float64MultiArray
        self.array = Float64MultiArray()
        self.array.data = [] # [time, j1_pos, j1_vel, j1_eff
                             # j2_pos, j2_vel, j2_eff
                             # ee_x, ee_y, ee_z]
        # Initiate subscriber with call_back
        self.joint_state_subscriber = rospy.Subscriber('/planar_robot/joint_states',
                                    JointState, self.joint_state_callback)

    def joint_state_callback(self, joint_data):
        # Get EE data
        try:
            (link2_pos, link2_rot) = self.listener.lookupTransform('base_link', 'link_2', 
                                                rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Listener didn't work")
        
        rot_matrix = transformations.quaternion_matrix(link2_rot)
        ee_from_link2 = np.array([0, 0, 0.25, 1]) # 0.25 is the length of link2
        ee_from_link2_in_base = np.dot(rot_matrix, ee_from_link2)
        ee_from_link2_in_base = ee_from_link2_in_base[:-1] # remove the last element i.e. 1
        ee_from_base = link2_pos + ee_from_link2_in_base

        # Set time
        time_secs = joint_data.header.stamp.secs
        time_nsecs = joint_data.header.stamp.nsecs
        time = time_secs + time_nsecs / 1e9
        # Set the array data
        # self.array.data.clear()
        self.array.data = [time, 
                joint_data.position[0], joint_data.velocity[0], joint_data.effort[0],
                joint_data.position[1], joint_data.velocity[1], joint_data.effort[1],
                ee_from_base[0], ee_from_base[1], ee_from_base[2]]
        
        # Publish array data
        self.data_publisher.publish(self.array)
        
    


if __name__ == "__main__":
    try:
        record = DataRecorder()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass