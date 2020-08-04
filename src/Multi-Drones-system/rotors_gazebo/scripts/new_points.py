#!/usr/bin/env python


# Waypoint publisher for RotorS
# Reference: https://github.com/ethz-asl/rotors_simulator/issues/510
# 

import rospy
import sys
import tf
import numpy as np

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Transform
from nav_msgs.msg import Odometry
# X=0
# Y=0

# def move_in_x(msg):
# 	if msg.pose.pose.orientation.z >-0.1 and msg.pose.pose.orientation.z <0.1  :
# 		dir =0
# 	if msg.pose.pose.orientation.z >= 0.65 and msg.pose.pose.orientation.z <0.75 :
# 		dir =1
# 	if msg.pose.pose.orientation.z >0.9 and msg.pose.pose.orientation.z <1.1 :
# 		dir =2
# 	if msg.pose.pose.orientation.z >-0.8 and  msg.pose.pose.orientation.z <-0.6 :
# 		dir =3
# 	if dir == 0: 
# 		publish_waypoint(msg.pose.pose.position.x+0.5, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.z)
# 	if dir == 1:
# 		publish_waypoint(msg.pose.pose.position.x, msg.pose.pose.position.y+0.5, msg.pose.pose.position.z, msg.pose.pose.orientation.z)
# 	if dir ==2:
# 		publish_waypoint(msg.pose.pose.position.x-0.5, msg.pose.pose.position.y, msg.pose.pose.position.z, msg.pose.pose.orientation.z)
# 	if dir ==3:
# 		publish_waypoint(msg.pose.pose.position.x, msg.pose.pose.position.y-0.5, msg.pose.pose.position.z, msg.pose.pose.orientation.z)
def callback(msg):
	#move_in_x(msg)
	publish_waypoint(msg.pose.pose.position.x, msg.pose.pose.position.y, 2, 0)

	

def publish_waypoint(x,y,z,yaw):
	"""
	Publish a waypoint to 
	"""

	command_publisher = rospy.Publisher('/iris/command/trajectory', MultiDOFJointTrajectory, queue_size = 10)

	# create trajectory msg
	traj = MultiDOFJointTrajectory()
	traj.header.stamp = rospy.Time.now()
	traj.header.frame_id = 'frame'
	traj.joint_names.append('base_link')


	# create start point for trajectory
	transforms = Transform()
	transforms.translation.x = 0
	transforms.translation.y = 0
	transforms.translation.z = 0
	
	quat = tf.transformations.quaternion_from_euler(yaw*np.pi/180.0, 0, 0, axes = 'rzyx')
	transforms.rotation.x = quat[0]
	transforms.rotation.y = quat[1]
	transforms.rotation.z = quat[2]
	transforms.rotation.w = quat[3]
	
	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
	traj.points.append(point)

	# create end point for trajectory
	transforms = Transform()
	transforms.translation.x = x
	transforms.translation.y = y
	transforms.translation.z = z 

	quat = tf.transformations.quaternion_from_euler(yaw*np.pi/180.0, 0, 0, axes = 'rzyx')
	transforms.rotation.x = quat[0]
	transforms.rotation.y = quat[1]
	transforms.rotation.z = quat[2]
	transforms.rotation.w = quat[3]

	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
	traj.points.append(point)

	rospy.sleep(1)
	command_publisher.publish(traj)


	

if __name__ == '__main__':
	try:
		rospy.init_node("riseq_rotors_waypoint_publisher", anonymous = True)

		odom_sub = rospy.Subscriber('/iris/odometry_sensor1/odometry', Odometry, callback)
		rospy.sleep(1)
		# # print("this",X)
		# x_des = 0.0
		# y_des = 5.0
		# z_des = 5.0
		# yaw_des = 90.0

		# publish_waypoint(x_des, y_des, z_des, yaw_des)
		# listen = rospy.Subscriber(' /iris/odometry_sensor1/odometry', Odometry, queue_size = 1)
		# newlisten = Odometry()
		# print(newlisten.pose)
		#rospy.init_node('check_odometry')
		#print(odom_sub.msg.pose)
		#print()
		rospy.spin()
		# rospy.loginfo(" >> Published waypoint: x: {}, y: {}, z: {}, yaw: {}".format(x_des, y_des, z_des, yaw_des))


	except rospy.ROSInterruptException:
		print("ROS Terminated")
		pass
