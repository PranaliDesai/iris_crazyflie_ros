#!/usr/bin/env python

import rospy
import sys
import tf
import numpy as np
import sys
import math
import rospkg
import yaml 

#mavros_msgs/PositionTarget
from geometry_msgs.msg import PoseStamped

def move():
    rospy.init_node('robot_cleaner', anonymous=False)
    #velocity_publisher = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=1)
    velocity_publisher = rospy.Publisher('/iris/command/pose', PoseStamped, queue_size=1)
    
    vel_msg = PoseStamped()

    vel_msg.pose.position.x = 0.6
    vel_msg.pose.position.y = 0
    vel_msg.pose.position.z = 1
    vel_msg.pose.orientation.z=1
    vel_msg.pose.orientation.w=1.57

    while not rospy.is_shutdown():
        velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        move()
    except rospy.ROSInterruptException: pass