#!/usr/bin/env python2.7
import rospy
import math
import tf
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import MagneticField
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import pymap3d as pm
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from rospy.timer import Rate
from std_msgs.msg import UInt16
from obstacle_detector.msg import Obstacles

from obstacle_detector.msg import CircleObstacle
from sensor_msgs.msg import LaserScan

class pathplanner():
    def __init__(self):
        rospy.Subscriber('/gps_data/fix',NavSatFix,self.GPS_IMU)
        rospy.Subscriber('/imu/mag',MagneticField,self.IMU_Calculation)
        rospy.Subscriber('/obstacles',Obstacles, self.Obstacle_detector)
        self.thrusterL_pub=rospy.Publisher('/thrusterL',UInt16,queue_size=10)
        self.thrusterR_pub = rospy.Publisher('/thrusterR', UInt16, queue_size=10)
        self.servo_pub=rospy.Publisher('/servopwm',UInt16,queue_size=10)
        self.cur_x=0.0
        self.cur_y=0.0
        self.yaw=0.0
        '''self.wp=np.array([[37.44805368566385, 126.65354953461339,0],
                          [37.448147990219944, 126.65359658447464,0],
                           [37.44800524136721, 126.6534670566824,0]])'''

        self.wp=np.array([[37.448128032084945, 126.65350582164348,0]
                        ,[37.44806521426223, 126.65353666704857,0]
                        ,[37.44798987719215, 126.65345082395835,0]])
        self.waypoints=np.array([[0,0,0]])
        self.obstacles=[]
        self.obob=[]
        self.arranged_obstacles=[]
        self.i=1
        self.tolerance=1.0
        self.ex_waypoint=[]
        self.dist_to_goal=30
        self.base_position=[37.44800379436486, 126.65340042517397,0.0]
        self.PI=math.pi
        self.waypoint=[]
        self.magnetic_x = 0.0
        self.magnetic_y = 0.0
        self.magnetic_z = 0.0
        self.mAzimuth = 0.0
        self.shookshook=[100,0]
        self.Ke=0.0
        self.Kp=0.0

    def GPS_IMU(self,data):

        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z
        self.cur_x, self.cur_y, u = pm.geodetic2enu(lat, lon, alt, self.base_position[0], self.base_position[1], self.base_position[2])

        
    def IMU_Calculation(self,data):
        self.magnetic_x = data.magnetic_field.x
        self.magnetic_y = data.magnetic_field.y
        self.magnetic_z = data.magnetic_field.z

    def Obstacle_detector(self,data):
        for i in data.circles:
            self.obstacles.append([i.center.y,-i.center.x,i.radius])
        
    
    def getRotationMatrix(self,R, I, gravity, geomagnetic):
        Ax = gravity[0]
        Ay = gravity[1]
        Az = gravity[2]

        normsqA = (Ax * Ax + Ay * Ay + Az * Az)
        g = 9.81
        freeFallGravitySquared = 0.01 * g * g

        if (normsqA < freeFallGravitySquared):
            # gravity less than 10% of normal value
            return False

        Ex = geomagnetic[0]
        Ey = geomagnetic[1]
        Ez = geomagnetic[2]
        Hx = Ey * Az - Ez * Ay
        Hy = Ez * Ax - Ex * Az
        Hz = Ex * Ay - Ey * Ax
        normH = math.sqrt(Hx * Hx + Hy * Hy + Hz * Hz)

        if normH < 0.1:
            # device is close to free fall (or in space?), or close to
            # magnetic north pole. Typical values are  > 100.
            return False

        invH = 1.0 / normH
        Hx *= invH
        Hy *= invH
        Hz *= invH

        invA = 1.0 / math.sqrt(Ax * Ax + Ay * Ay + Az * Az)
        Ax *= invA
        Ay *= invA
        Az *= invA

        Mx = Ay * Hz - Az * Hy
        My = Az * Hx - Ax * Hz
        Mz = Ax * Hy - Ay * Hx
        if (R != None) :
            if len(R) == 9:
                R[0] = Hx;     R[1] = Hy;     R[2] = Hz
                R[3] = Mx;     R[4] = My;     R[5] = Mz
                R[6] = Ax;     R[7] = Ay;     R[8] = Az
            elif len(R) == 16:
                R[0]  = Hx;    R[1]  = Hy;    R[2]  = Hz;   R[3]  = 0
                R[4]  = Mx;    R[5]  = My;    R[6]  = Mz;   R[7]  = 0
                R[8]  = Ax;    R[9]  = Ay;    R[10] = Az;   R[11] = 0
                R[12] = 0;     R[13] = 0;     R[14] = 0;    R[15] = 1
            
        if (I != None):
            # compute the inclination matrix by projecting the geomagnetic
            # vector onto the Z (gravity) and X (horizontal component
            # of geomagnetic vector) axes.
            invE = 1.0 / math.sqrt(Ex * Ex + Ey * Ey + Ez * Ez)
            c = (Ex * Mx + Ey * My + Ez * Mz) * invE
            s = (Ex * Ax + Ey * Ay + Ez * Az) * invE

            if (len(I) == 9):
                I[0] = 1;     I[1] = 0;     I[2] = 0
                I[3] = 0;     I[4] = c;     I[5] = s
                I[6] = 0;     I[7] = -s;     I[8] = c
            elif (len(I) == 16):
                I[0] = 1;     I[1] = 0;     I[2] = 0
                I[4] = 0;     I[5] = c;     I[6] = s
                I[8] = 0;     I[9] = -s;     I[10] = c
                I[3] = I[7] = I[11] = I[12] = I[13] = I[14] = 0
                I[15] = 1

        return True
        
    def getOrientation(self,R, values):
        if (len(R) == 9):
            values[0] = math.atan2(R[1], R[4])
            values[1] = math.asin(-R[7])
            values[2] = math.atan2(-R[6], R[8])
        else:
            values[0] = math.atan2(R[1], R[5])
            values[1] = math.asin(-R[9])
            values[2] = math.atan2(-R[8], R[10])
        
        return values

    def calculation(self):
        mAzimuth=0
        gravity = [0, 0, -9.8]
        geomagnetic = [self.magnetic_x *  10**6, self.magnetic_y *  10**6, self.magnetic_z *  10**6] 
        R = [0]*9
        orientation = [0] * 3
        success = self.getRotationMatrix(R, None, gravity, geomagnetic)
        if success is True:
            self.getOrientation(R, orientation)
            mAzimuth = orientation[0] * 180 / math.pi
            #mPitch = orientation[1] * 180 / math.pi
            #mRoll = orientation[2] * 180 / math.pi

        self.yaw = round(2*mAzimuth,0)
        self.yaw = self.yaw/2      

    def waypoint_maker(self):
        for k in self.wp:
            e,n,u = pm.geodetic2enu(k[0],k[1],k[2], self.base_position[0], self.base_position[1], self.base_position[2])
            self.waypoints=np.append(self.waypoints,np.array([[e,n,u]]),axis=0)
           
    
    def waypoint_step(self):
        if self.i != len(self.waypoints):
            if self.dist_to_goal > self.tolerance:
                self.waypoint = self.waypoints[self.i]  # not arrived
                self.ex_waypoint = self.waypoints[self.i - 1]
                    
            elif self.dist_to_goal <= self.tolerance:
                self.i = self.i + 1  # arrived
                self.waypoint = self.waypoints[self.i]
                self.ex_waypoint = self.waypoints[self.i-1]
               


        elif self.i + 1 == len(self.waypoints):
            self.waypoint = self.waypoints[self.i]  # no more waypoint
            self.ex_waypoint = self.waypoints[self.i - 1]
            
            if self.dist_to_goal <= self.tolerance:
                
                pass
                
        return self.waypoint

    def Distance_To_Goal(self,waypoint):
        error_x = waypoint[0] - self.cur_x
        error_y = waypoint[1] - self.cur_y

        theta = math.atan2(error_x, error_y) * 180 / math.pi #y axis = north x/y
        
        error_N = theta - self.yaw
        if error_N>180:
            error_N-=360  

        self.dist_to_goal = math.sqrt(math.pow(error_x,2)+ math.pow(error_y,2))
        
        return error_N 

    def dist_to_obstacles(self):
        for i in self.obstacles:
            dist_ob=math.sqrt(math.pow((i[0]-self.cur_x),2)+math.pow((i[1]-self.cur_y),2))-i[2]
            angle_ob_st=math.degrees(math.atan2((i[0]-self.cur_x),(i[1]-self.cur_y)))-math.degrees(math.atan2(dist_ob,i[2]))
            angle_ob_ed=math.degrees(math.atan2((i[0]-self.cur_x),(i[1]-self.cur_y)))+math.degrees(math.atan2(dist_ob,i[2]))
            self.obob.append([angle_ob_st,angle_ob_ed,dist_ob])
        

    def global_path(self,error_N):
        print('global_path')
        self.thrusterL_pub.publish(1500-self.dist_to_goal*50)
        self.thrusterR_pub.publish(1500+self.dist_to_goal*50)
        self.servo_pub.publish(90+error_N*0.1)
        time.sleep(0.1)

    #self.obstacles=[[x,y,r],[x,y,r]...]
    #obob=[[dist,angle]]        
    def local_path(self,error_N):
        print('local_path')
        #need obstacle update
        safe_zone=[]
        for i in self.obob:
            time.sleep(0.1)    





                
                    
                
                


pp=pathplanner()
def main():
    rospy.init_node("KD",anonymous=True)
    pp.waypoint_maker()
    while not rospy.is_shutdown():
        #print(pp.shookshook)
        pp.dist_to_obstacles()
        pp.calculation()
        waypoint=pp.waypoint_step()
        error_N = pp.Distance_To_Goal(waypoint)
        print(pp.obstacles)
        if pp.shookshook[0]>5:
            pp.global_path(error_N)
        else:
            pp.local_path(error_N)
            

            

    rospy.spin()

if __name__ == '__main__':
     main()


            


         



        
    
    
