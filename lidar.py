#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
from lidar_tutorials.msg import new

i = 0

def msg_callback(msg):
    print(msg.header.stamp)
    temp = new()
    global i
    temp.x = i*2
    temp.y = i*2
    for p in range(len(msg.ranges)):
        temp.distance.append(msg.ranges[p])
    i += 1
    pub.publish(temp)

rospy.init_node('subscriber1', anonymous=True)

pub = rospy.Publisher("lidar_go", new)
while not  rospy.is_shutdown() :
    rospy.Subscriber('scan', LaserScan, msg_callback)
    rospy.spin()
