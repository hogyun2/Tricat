#!/usr/bin/env python
#-*- coding:utf-8 -*-

from __future__ import division, print_function         #파이썬3문법을 2에서도 쓸수있게해줌
import serial
import rospy
import math
import tf
import numpy as np
from std_msgs.msg import Time
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from ublox_msgs.msg import NavPVT
from sensor_msgs.msg import Imu
import pymap3d as pm
import time
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from rospy.timer import Rate
from std_msgs.msg import UInt16
import threading
from std_msgs.msg import Int64

#define
magnet_OFF = [0, 0, 0]
PI = 3.141592

#ENU base
base_lat =37.44800418820753
base_lon =126.65339929495622
base_alt =0.0

class pathplanner():
    def __init__(self):
        rospy.Subscriber('/gps_data/fix',NavSatFix,self.GPS_call)
        rospy.Subscriber('/gps_data/navpvt',NavPVT,self.GPS_Heading)        #gps heading(hogyun)
        rospy.Subscriber('/imu/data',Imu,self.IMU_Calculation)
        rospy.Subscriber('/timer', Time, self.Time_call)                    #timer(hogyun)
        #rospy.Subscriber('/obstacles',)
        self.thruster_pub=rospy.Publisher('/thrusterpwm',UInt16,queue_size=10)
        self.servo_pub=rospy.Publisher('/servopwm',UInt16,queue_size=10)
        self.e_gps = None #cur_X
        self.n_gps = None #cur_y
        self.u_gos = None
        self.yaw_gps = 0 #gps heading
        self.yaw_imu = 0 #imu heading
        self.yaw_final = None #offset 보정 후 heading4

        '''self.wp=np.array([[37.44805368566385, 126.65354953461339,0],
                          [37.44810372699222, 126.65349454932608,0],
                           [37.44800524136721, 126.6534670566824,0]])'''

        self.wp=np.array([[37.448053164859516, 126.65349786614203,0]
                        ,[37.44797863515896, 126.65349116061918,0]])
        self.waypoints=np.array([[0.0,0.0,0.0]])
        self.obstacles=np.array([[0.0,0.0]])
        self.i=1
        self.tolerance=5.0
        self.ex_waypoint=[]
        self.dist_to_goal=30.0
#        self.base_position=[37.44800418820753, 126.65339929495622,0.0]
        self.PI=math.pi

        self.offset = 0

        self.t_Time_call = 0
        self.t_GPS_call = None
        self.t_IMU_call = 0

        self.heading = 0
        self.gps_out = 0
        self.imu_out = 0
        self.headingAcc = 99999999


    #msg 선언
    def msg_write(self,msg):
        self.msg.pose.pose.position.x = float(self.e_gps)
        self.msg.pose.pose.position.y = float(self.n_gps)
        self.msg.pose.pose.position.z = float(self.u_gps)
        self.msg.twist.twist.angular.z = self.yaw_final


    #gps 좌표
    def GPS_call(self,data):
        self.t_GPS_call = t.time()

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.e_gps,self.n_gps,self.u_gps = pm.geodetic2enu(lat,lon,alt,base_lat,base_lon,base_alt)

    #gps heading 구하는 부분
    def GPS_Heading(self, data):
        self.yaw_gps = (450-(data.heading * 10**(-5)))%360
        self.hAcc = data.hAcc
        self.headingAcc =data.headAcc


    # imu 
    def IMU_call(self, data):
        self.t_IMU_call = t.time()
        quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw_imu = (euler[2] * 180 / PI + 90)         # Yaw

        print('yaw',self.yaw_imu)

    def decide_heading(self):
        if self.imu_out == 0:
            if self.gps_out == 0:
                if self.headingAcc<300000: #gps o,imu o & 신뢰도 o
                    self.offset = self.yaw_gps - self.yaw_imu 
                    # self.yaw_final = self.yaw_imu + self.offset
                    # print("RTK ON, Reliabilty good, OFFSET ON")
                    self.yaw_final = self.yaw_imu + self.offset                  

                else: #gps o,imu o & 신뢰도 x
                    self.yaw_final = self.yaw_imu + self.offset
                    # print("RTK OFF, Reliabilty good, OFFESET OFF")
            else:
                self.yaw_final = self.yaw_imu + self.offset
                # print("RTK OFF Reliabilty bad, OFFESET OFF")
        
        else:
            if self.gps_out == 0:
                self.yaw_final = self.yaw_gps
                # print("GPS Heading")
            else:
                # print("LANE DETECTION")
                pass
        # print("imu yaw : ",self.yaw_imu)
        # print("gps heading : ", self.yaw_gps)
        #self.yaw_final = self.yaw_gps        
        

    def Time_call(self,time):
        self.t_Time_call = t.time()

        if self.t_Time_call - self.t_GPS_call > 0.2:
            self.gps_out = 1
        else:
            self.gps_out = 0

        if self.t_Time_call - self.t_IMU_call > 0.5:
            self.imu_out = 1
        else:
            self.imu_out = 0  

        self.decide_heading()

        self.yaw_final = self.yaw_final%360

        self.msg_write(self.msg)
        print("I:",round(self.yaw_imu),"  g:",round(self.yaw_gps),"   O: ",round(self.offset),"  f:",round(self.yaw_final))
        self.msg.header.stamp = rospy.Time.now()
        self.pub.publish(self.msg)   

        
    def waypoint_maker(self):
        for i in self.wp:
            e,n,u = pm.geodetic2enu(i[0],i[1],i[2], self.base_position[0], self.base_position[1], self.base_position[2])
            self.waypoints=np.append(self.waypoints,np.array([[e,n,u]]),axis=0)
    
    def waypoint_step(self):
        if self.i + 1 != len(self.waypoints):
            if self.dist_to_goal > self.tolerance:
                if self.i==0:
                    waypoint = self.waypoints[self.i]
                    self.ex_waypoint=[0.0,0.0]
                else:
                    waypoint = self.waypoints[self.i]  # not arrived
                    self.ex_waypoint = self.waypoints[self.i - 1]

            elif self.dist_to_goal <= self.tolerance:
                waypoint = self.waypoints[self.i + 1]
                self.ex_waypoint = self.waypoints[self.i]
                self.i = self.i + 1  # arrived


        elif self.i + 1 == len(self.waypoints):
            waypoint = self.waypoints[self.i]  # no more waypoint
            self.ex_waypoint = self.waypoints[self.i - 1]
            if self.dist_to_goal <= self.tolerance:
                pass
                #print("GOAL!")
        print(waypoint)
        
        return waypoint

    def Distance_To_Goal(self,waypoint):
        error_x = waypoint[0] - self.e_gps
        error_y = waypoint[1] - self.n_gps

        theta = math.atan2(error_x, error_y) * 180 / math.pi #y axis = north x/y
        
        error_N = theta + self.yaw  
        
        if 180 <= error_N and error_N < 360:
            error_N = error_N - 360
        elif -360 < error_N and error_N < -180:
            error_N = error_N + 360

        self.dist_to_goal = math.sqrt(error_x * error_x + error_y * error_y)
        return error_N 

    def global_path(self,error_N):
        if abs(error_N)<10:
            self.thruster_pub.publish(1600)
            self.servo_pub.publish(90)
            time.sleep(1)

        elif 10<=abs(error_N)<90:
            self.thruster_pub.publish(1600)
            if error_N>0:
                self.servo_pub.publish(80)
            else:
                self.servo_pub.publish(100)
            time.sleep(1)

        elif 90<=abs(error_N)<=180:
            self.thruster_pub.publish(1600)
            if error_N>0:
                self.servo_pub.publish(70)
            else:
                self.servo_pub.publish(110)
            time.sleep(1)
    '''def local_path(self,dist_to_goal,error_N,waypoint)
        dist_to_line=(cur_x*(waypoint[1]-self.ex_waypoint[1])-self.cur_y*(waypoint[0]-self.ex_waypoint[0])-self.ex_waypoint[0]*(waypoint[1]-self.ex_waypoint[1])+self.ex_waypoint[1]*(waypoint[0]-self.ex_waypoint[0]))
                     /(np.sqrt(np.power((waypoint[1]-self.ex_waypoint[1]),2)+np.power((waypoint[0]-self.ex_waypoint[0]),2)))
        for i in self.obstacles:
            dist_to_ob.append()'''


pp=pathplanner()
def main():
    rospy.init_node("KD",anonymous=True)
    pp.waypoint_maker()
    while not rospy.is_shutdown():
        waypoint=pp.waypoint_step()
        print('wp',waypoint)
        error_N = pp.Distance_To_Goal(waypoint)
        if len(pp.obstacles)==0:
            pp.local_path()
            
        else:
            pp.global_path(error_N)
            #print("error_N",error_N)
            #print("Yaw",pp.yaw)
            

            

    rospy.spin()

if __name__ == '__main__':
     main()


            


         



        
    
    
