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
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


from sensor_msgs.msg import LaserScan
multiranger_data =(0,0,0,0)
x=0
y=0
z=0
dir=0

def rotate_left():
	global dir
	global x
	global y
	global z
	if dir ==0:
		publish_waypoint(x , y,2, 90)
	elif dir ==1:
		publish_waypoint(x , y,2, 180)
	elif dir ==2:
		publish_waypoint(x , y,2, 270)
	elif dir ==3:
		publish_waypoint(x , y,2, 0)
	dir=(dir+1)%4

def rotate_right():
	global dir
	global x
	global y
	global z
	if dir ==0:
		publish_waypoint(x , y,2, 270)
	elif dir ==1:
		publish_waypoint(x , y,2, 0)
	elif dir ==2:
		publish_waypoint(x , y,2, 90)
	elif dir ==3:
		publish_waypoint(x , y,2, 180)
	dir=(dir+3)%4


def move(a,b,c):
	global dir
	global x
	global y
	global z
	if dir ==0:
		publish_waypoint(x+a ,y+b,2+c, 0)
	if dir ==1:
		publish_waypoint(x-b , y+a,2+c, 90)
	if dir == 2:
		publish_waypoint(x-a, y-b,2+c, 180)
	if dir ==3:
		publish_waypoint(x+b , y-a,2+c, 270)



def listener():
	global multiranger_data
	global x
	global y
	global z
	global dir
	rospy.sleep(1)
	print("dirrrr---",dir)
	rotate_right()
	# move(0,0.4,0)
	# if (multiranger_data[2]>5):
	# 	publish_waypoint(x+0.2 , y, 2, 0)
	

def callback1(msg):
	global multiranger_data
	multiranger_data = msg.ranges


def callback(msg):
	global x
	global y 
	global z
	x=msg.point.x
	y=msg.point.y
	z=msg.point.z

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

	quat = tf.transformations.quaternion_from_euler((yaw)*np.pi/180.0, 0, 0, axes = 'rzyx')
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

rospy.init_node("riseq_rotors_waypoint_publisher", anonymous = True)
sub = rospy.Subscriber('/iris/LaserScan', LaserScan, callback1)
odom_sub = rospy.Subscriber('/iris/ground_truth/position', PointStamped, callback)
while not rospy.is_shutdown():
	# rospy.sleep(1)
	listener()

	#rospy.spin()


