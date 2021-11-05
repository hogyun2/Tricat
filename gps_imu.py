#!/usr/bin/env python
import rospy
import math
import tf
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import pymap3d as pm
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from rospy.timer import Rate
from std_msgs.msg import Float64

# define
magnet_OFF = [0, 0, 0]
PI = 3.141592
msg = Odometry()

#settings_KD
lat=NavSatFix.latitude
lon=NavSatFix.longitude
alt=NavSatFix.altitude

# # base latlon
base_lat = 37.4505348
base_lon = 126.6529716
base_alt = 22


def get_xy(lat, lon, alt):
    e, n, u = pm.geodetic2enu(lat, lon, alt, base_lat, base_lon, base_alt)
    return e, n, u



count = 0

# fuction
def GPS_IMU(data):

    lon = float(data.longitude) #x
    lat = float(data.latitude) #y
    alt = float(data.altitude) #z

    e, n, u = get_xy(lat, lon, alt)

    msg.pose.pose.position.y = float(data.latitude)
    msg.pose.pose.position.x = float(data.longitude)

    msg.pose.pose.position.x = float(n)
    msg.pose.pose.position.y = float(e)
    msg.twist.twist.linear.x = float(time.time()) 
    #print('msg : ', msg)
    

    pub.publish(msg)
    



def IMU_Calculation(data):
    quaternion = (data.orientation.x,
        data.orientation.y,
        data.orientation.z,
        data.orientation.w)

    euler = tf.transformations.euler_from_quaternion(quaternion)
    msg.twist.twist.angular.z = euler[2]*180/PI # Yaw
    msg.twist.twist.angular.y = euler[1]*180/PI # Yaw
    msg.twist.twist.angular.x = euler[0]*180/PI # Yaw
    #yaw_imu = -data.orientation.x
    #yaw_imu = yaw_imu % 360
    print(msg.twist.twist.angular.z)
    msg.pose.pose.position.z = msg.twist.twist.angular.z
    if (msg.pose.pose.position.z > 180): 
        msg.pose.pose.position.z -= 360
    
#print('msg : ', msg.pose.pose.position.z)
pub = rospy.Publisher('/pose', Odometry, queue_size = 1)
rospy.init_node("Position_Node" , anonymous=True)
rospy.Subscriber('/gps_data/fix',NavSatFix,GPS_IMU)
rospy.Subscriber('/imu/data',Imu,IMU_Calculation)
rate = rospy.Rate(1000)
rospy.spin()
