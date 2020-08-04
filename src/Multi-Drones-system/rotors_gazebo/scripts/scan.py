#! /usr/bin/env python

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from rotors_msgs.msg import min_val
import sys
import heapq

first_arg=sys.argv[1]
count=20
pub= rospy.Publisher('/{}/min_vals'.format(first_arg),min_val,queue_size = 1)
#print"in-------",(LaserScan)
def callback(msg):
    listtheta=[]
    listvalue=[]
    finmsg=min_val()
    # mintheta = np.argmin(msg.ranges)
    # minvalue = msg.ranges[mintheta]
    values=list(map(lambda n: (n, (msg.ranges).index(n)), heapq.nsmallest(count, msg.ranges)))
    # print(values)
    for i in range(count):
        listvalue.append(values[i][0])
        listtheta.append(values[i][1])
        print(values[i][1])
    finmsg.distance.data = listvalue
    finmsg.angle.data = listtheta 
    # rospy.loginfo(finmsg)
    pub.publish(finmsg)
 
rospy.init_node('scan_values')
sub = rospy.Subscriber('/{}/LaserScan'.format(first_arg), LaserScan, callback)
rospy.sleep(0.5)
rospy.spin()
