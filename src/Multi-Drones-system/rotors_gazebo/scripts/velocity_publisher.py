#!/usr/bin/env python

# Waypoint publisher for RotorS
# Reference: https://github.com/ethz-asl/rotors_simulator/issues/510
# 

import rospy
import sys
import tf
import numpy as np
import sys
import math
import rospkg
import yaml 

from planner_final import robgoals
# from tsp_11 import robgoals


from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Transform
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Int32MultiArray
from rotors_msgs.msg import min_val
first_arg = sys.argv[1]



class vel_publisher():
    def __init__(self,goals):
        global first_arg
    	rospy.init_node('rotors_goal_publisher_{}'.format(first_arg))
     	# self.goals = np.array(([[0,1,3,0],[0,1,3,0],[0,4,3,0],[-2,4,3,0],[-2,6,3,0],[-4,8,3,0]]))
     	self.sub = rospy.Subscriber('/{}/ground_truth/position'.format(first_arg), PointStamped , self.position_status)
     	self.sub = rospy.Subscriber('/{}/min_vals'.format(first_arg), min_val , self.obstacle_avoid,queue_size=1)
        self.command_publisher = rospy.Publisher('/{}/command/trajectory'.format(first_arg), MultiDOFJointTrajectory, queue_size=1)
     	self.iter_local=2
     	self.iter_global=0
      	self.goals=goals
        self.X_position=None
        self.Y_position=None
        self.Z_position=None
        self.goal_angle=0
        self.threshold_angle=360
        self.threshold_distance=0
        self.flag=0
        self.goal_list=self.final_list(4,self.goals[0],self.goals[1])
        self.lenght=0

    def publish_waypoint(self,x,y,z,yaw):
        
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = 'frame'
        traj.joint_names.append('base_link')
        # Create start point
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

        velocities=Twist()
        accel = Twist()
        point = MultiDOFJointTrajectoryPoint([transforms],[velocities],[accel],rospy.Time(2))
        traj.points.append(point)
        rospy.sleep(0.01)
        self.command_publisher.publish(traj)
    
    def distance_formula(self,X1,Y1,X2,Y2):
        distance=((X1-X2)**2+(Y1-Y2)**2)**(0.5)
        return distance
    
    def obstacle_avoid(self,msg):
        distance_mat=msg.distance.data
        angle_mat=msg.angle.data
        danger_distance=[]
        danger_angle=[]
        if (self.flag==1 and self.iter_global<len(self.goals)-1):
            self.goal_list=self.final_list(3,self.goals[self.iter_global],self.goals[self.iter_global+1])
            self.iter_local=0
            self.flag=0
        # calculate number of danger points
        for i in range (len(angle_mat)):
            if distance_mat[i]==np.inf:
                L1=list(distance_mat)
                L1[i]=1000
                distance_mat=tuple(L1)

            if distance_mat[i]<self.threshold_distance: #and angle_mat[i]<self.threshold_angle/2+self.goal_angle and angle_mat[i]>(self.goal_angle-self.threshold_angle/2+360)%360:
                danger_distance.append(distance_mat[i])
                danger_angle.append(angle_mat[i])

        if danger_distance==[]:
            if self.iter_local<self.goal_list.shape[0]:
                print('goal shape',self.goal_list.shape[0])
                x_des = float(self.goal_list[self.iter_local,0])
                y_des = float(self.goal_list[self.iter_local,1])
                z_des = float(self.goal_list[self.iter_local,2])
                yaw_des = float(self.goal_list[self.iter_local,3])
                self.publish_waypoint(x_des, y_des, z_des, yaw_des)
                self.goal_angle=math.degrees(math.atan2((self.goal_list[self.iter_local,1]-self.Y_position),(self.goal_list[self.iter_local,0]-self.X_position)))
                if abs(self.goal_list[self.iter_local,0]-self.X_position)<0.2 and abs(self.goal_list[self.iter_local,1]-self.Y_position)<0.2 and abs(self.goal_list[self.iter_local,2]-self.Z_position)<0.2:
                    if self.iter_local+1<len(self.goal_list):
                        self.lenght+=self.distance_formula(self.goal_list[self.iter_local,0],self.goal_list[self.iter_local,1],self.goal_list[self.iter_local+1,0],self.goal_list[self.iter_local+1,1])
                        self.lenght+=self.distance_formula(0,self.goal_list[self.iter_local,2],0,self.goal_list[self.iter_local+1,2])

                    self.iter_local+=1
                if self.iter_local==len(self.goal_list) and abs(self.Z_position-2)<0.2:
                    self.flag=1
                    self.iter_global+=1
                rospy.sleep(0.2)
                if self.Z_position<2.5:
                    rospy.sleep(2)
                #rospy.spinOnce()
                rospy.loginfo(" >> Published waypoint: x: {}, y: {}, z: {}, yaw: {}".format(x_des, y_des, z_des, yaw_des))
                rospy.loginfo(" >> Length of flight:{} of Drone : {}".format(self.lenght, first_arg))
        else:
            print("Obstacle ahead")
            x_des = float(self.X_position)
            y_des = float(self.Y_position)
            z_des = float(self.Z_position)
            yaw_des = float(0)
            current=np.array(([x_des,y_des,z_des,yaw_des]))
            self.goal_list=self.final_list(z_des+3,current,self.goals[self.iter_global+1])
            self.iter_local=0
            self.publish_waypoint(x_des, y_des, z_des, yaw_des)
            rospy.sleep(2)
        # print(distance_mat,'distance mat')
        # print(angle_mat,'angle mat')
        if self.iter_global>=len(self.goals)-1:
            rospy.loginfo(" >> Length of flight:{} of Drone : {}".format(self.lenght, first_arg))
            
    def position_status(self,data):
    	self.X_position=data.point.x
     	self.Y_position=data.point.y
      	self.Z_position=data.point.z

    def final_list(self,height,goal_start,goal_end):
        h = []
        g1 = goal_start
        g2 = goal_end
        interval=int(max(abs(g2[1]-g1[1]),abs(g2[0]-g1[0])))
        if abs(float(g2[0])-float(g1[0]))<0.1 :
            m=1000
        else:
            m = ((float(g2[1])-float(g1[1]))/(float(g2[0])-float(g1[0])))
        
        b = g1[1]-(m*g1[0])
        list_1 = np.linspace(g1[0],g2[0], num =interval*2)
        print(interval)
        for i in list_1:
            y = m*i + b
            node = [i, y, height, 0]
            h.append(node)
        h.append([g2[0], g2[1], 2, 0])
        h = np.array((h))
        return h

def import_yaml():
    rospack = rospkg.RosPack()
    with open(rospack.get_path('rotors_gazebo')+"/goals/goals.yaml", 'r') as stream:
        try:
            goal_locations=yaml.safe_load(stream)
            #print(bay_lib)
        except yaml.YAMLError as exc:
            print(exc)
            print("Check Global_planner.yaml in config folder")
        return goal_locations

if __name__ == '__main__':
    goals=np.array(([0,0,0,0]))
    goal_locs=import_yaml()
    print("rob",robgoals)
    for i in robgoals[int(first_arg[-1])-1]:
        goals=np.vstack((goals,(goal_locs['drone_goals']['G'+str(i)])))
    # To remove default zeros
    goal=goals[1:,:]
    # h=final_list(goal)
    goal = np.array((goal))
    print(goal)
    vel_publisher(goal)
    rospy.spin()