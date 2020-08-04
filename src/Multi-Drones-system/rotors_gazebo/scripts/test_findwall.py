#!/usr/bin/env python


# Waypoint publisher for RotorS
# Reference: https://github.com/ethz-asl/rotors_simulator/issues/510
# 

import rospy
import sys
import tf
import numpy as np
import matplotlib.pyplot as plt
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry


from sensor_msgs.msg import LaserScan
state_0 = 1
state_1 = 0
state_2 = 0 
state_3 = 0
multiranger_data =(0,0,0,0)
temp =[1000,1000]
x=0
y=0
z=0
val=0
dir=0


def follow_wall(range_f,range_l,state_1,state_2,temp, range_r,val):
    print("val===",val)
    plt.plot(x, y, color='green', marker='o', linestyle='dashed',linewidth=2, markersize=12)
    min_distance_left = 0.2
    max_distance_left = 0.3
    print("ranges", range_f,range_l)
    #yellow_pts(temp, range_r)
    if range_f < 0.9 and range_l < 1.1:  #corner 1 condition
        val+=1
        rotate_right()
        temp = [x, y]
        rospy.sleep(1)

    if abs(x-temp[0])>0.2 or abs(y-temp[1])>0.2:
        print("-------------------------")
        # c1=plt.Circle((temp[0],temp[1]), 0.13, color='b', fill=False)
        # ax.add_artist(c1)
        # wp.append(temp)
        temp = [x, y]

    # if range_f > 1 and range_l > 2:
    #     print("whye", range_l,range.f)
    #     for i in range(0,4):
    #         move(0.05,0,0)
    #         plt.plot(x, y, color='green', marker='o', linestyle='dashed',linewidth=2, markersize=12)
    #     rospy.sleep(0.1)
    #     rotate_left()
    #     temp = [x, y]
    #     rospy.sleep(0.1)
    #     state_1=0
    #     state_2=1
    if range_l < 1:
        move(0,-0.1,0)
    elif range_l> 1:
        move(0,0.1,0)
    move(0.2,0,0)
    if val == 4:
        plt.show()
    print("state_2",state_2)

    return state_1,state_2,temp,val

def find_wall(range,state_0, state_1):
    print(range,"------------")
    plt.plot(x, y, color='red', marker='o', linestyle='dashed',linewidth=2, markersize=12)
    if range > 1:
        move(0.2,0,0)
        rospy.sleep(0.5)
    elif range < 1:
        print("in----")
        rotate_right()
        state_0=0
        state_1=1
        rospy.sleep(1)
    return state_0, state_1

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
    global dir
    global multiranger_data
    global x
    global y
    global z
    global state_0
    global state_1
    global state_2
    global val
    yellow_point_list = []
    temp=[1000,1000]
    rospy.sleep(1)
    print(state_2)
    if state_0 == 1:
        state_0,state_1=find_wall(multiranger_data[2],state_0,state_1)
    if state_1 == 1:
        state_1,state_2,temp,val = follow_wall(multiranger_data[2],multiranger_data[1],state_1,state_2,temp, multiranger_data[3],val)
    if state_2 == 1:
        plt.show()

	

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
plt.show()


