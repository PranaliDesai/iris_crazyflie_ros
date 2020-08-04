#!/usr/bin/env python


# Waypoint publisher for RotorS
# Reference: https://github.com/ethz-asl/rotors_simulator/issues/510
# 

import rospy
import sys
import tf
import numpy as np
import sys
import yaml
from tsp import robgoals
import rospkg

first_arg = sys.argv[1]
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist 
from geometry_msgs.msg import Transform

def publish_waypoint(x,y,z,yaw,first_arg):
	"""
	Publish a waypoint to 
	"""
	print("{}".format(first_arg))

	command_publisher = rospy.Publisher('/{}/command/trajectory'.format(first_arg), MultiDOFJointTrajectory, queue_size = 10)

	# create trajectory msg
	traj = MultiDOFJointTrajectory()
	traj.header.stamp = rospy.Time.now()
	traj.header.frame_id = 'frame'
	traj.joint_names.append('base_link')


	# create start point for trajectory
	"""	
	transforms = Transform()
	velocities = Twist()
	accel = Twist()
	point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(1))	
	"""
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

	rospy.sleep(3)
	command_publisher.publish(traj)

def import_yaml():
    rospack = rospkg.RosPack()
    with open(rospack.get_path('rotors_gazebo')+"/goals/goals.yaml", 'r') as stream:
        try:
            goal_locations=yaml.safe_load(stream)
            #print(bay_lib)
        except yaml.YAMLError as exc:
            print(exc)
            print("check Global_planner.yaml in config folder")
        return goal_locations

def final_list(g1,g2):

		m = ((float(g2[1])-float(g1[1]))/(float(g2[0])-float(g1[0])))
		b = g1[1]-(m*g1[0])

		list = np.linspace(g1[0],g2[0], num = 20)
		h = []
		for i in list:
			
			y = m*i + b

			node = [i, y, 0.5, 0]

			h.append(node)
		return h
if __name__ == '__main__':
    goal=np.array(([0,0,0,0]))
    goal_locs=import_yaml()
    for i in robgoals[int(first_arg[-1])-1]:
        goal=np.vstack((goal,(goal_locs['drone_goals'][i])))
    # To remove default zeros

    goal=goal[1:,:]
    
    

    

    for i in range(len(goal)-1):
	g1 = goal[i]
	g2 = goal[i+1]
	

	h = final_list(g1, g2)
	goals = np.array((h))

    	try:
            rospy.init_node("rotors_waypoint_publisher", anonymous = True)

            for i in range(goals.shape[0]):
                x_des = float(goals[i,0])
                y_des = float(goals[i,1])
                z_des = float(goals[i,2])
                yaw_des = float(goals[i,3])

                publish_waypoint(x_des, y_des, z_des, yaw_des,first_arg)

                #rospy.spinOnce()
                rospy.loginfo(" >> Published waypoint: x: {}, y: {}, z: {}, yaw: {}".format(x_des, y_des, z_des, yaw_des))
    	except rospy.ROSInterruptException:
            print("ROS Terminated")
            pass

