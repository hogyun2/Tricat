#! /usr/bin/env python3.9
import math
from enum import Enum
from nav_msgs import msg
import rospy
import time
import matplotlib.pyplot as plt
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from rospy.timer import Rate
from std_msgs.msg import Float64
import tf
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import MagneticField
import pymap3d as pm
import time


class Pathplanner():
    """
    simulation parameter class
    """

    def __init__(self):
        rospy.Subscriber('/gps_data/fix_velocity',TwistWithCovarianceStamped,self.V_sub)
        rospy.Subscriber('/imu/data',Imu,self.W_sub)
        rospy.Subscriber('/gps_data/fix',NavSatFix,self.GPS_IMU)
        rospy.Subscriber('/imu/data',Imu,self.IMU_Calculation)
        # robot parameter
        self.to_goal_cost_gain = 0.15 #0.15
        self.speed_cost_gain = 1.0 #1.0
        self.obstacle_cost_gain = 1.0 #1.0
        self.base=[0.0,0.0,0.0]

        self.robot_stuck_flag_cons = 0.001
        self.robot_type = 1
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2
        self.cur_x=0.0
        self.cur_y=0.0
        self.cur_z=0.0
        self.yaw=0.0
        self.velocity = 0.0
        self.angular_velocity=0.0
        self.V_x=0.0
        self.V_y=0.0
        self.heading=0.0
        self.accel=()
        self.ob = np.array([[-1,-1]])
        self.ex_waypoint=[0.0,0.0]
        self.waypoints = np.array([[33.90706545703714, -23.517886336479854],
                                   [5,10],
                                   [10,10]])
        self.i=0
        self.tolerance=2
        self.dist_to_goal=30
        self.v_pub=rospy.Publisher('/V',Float64,queue_size=10)
        self.w_pub=rospy.Publisher('/W',Float64,queue_size=10)
        self.max_speed = 1.8  # [m/s]
        self.min_speed = -0.4  # [m/s]
        self.max_yaw_rate = 2 / 35 * math.pi  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        self.max_delta_yaw_rate = 2 / 35 * math.pi  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 2.0  # [s]
        self.PI=3.141592

        

    def GPS_IMU(self,data):
        lon = float(data.longitude) #x
        lat = float(data.latitude) #y
        alt = float(data.altitude) #z

        self.cur_y, self.cur_x, self.cur_z = pm.geodetic2enu(lat, lon, alt, self.base[0], self.base[1], self.base[2]) #x is north

    def IMU_Calculation(self,data):
        quaternion = (data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w)

        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = euler[2]*180/self.PI # Yaw
        if (self.yaw > 180): 
            self.yaw -= 360
    
    def V_sub(self,msg):
        self.V_x = msg.twist.twist.linear.x
        self.V_y = msg.twist.twist.linear.y
        self.velocity = math.sqrt(math.pow(self.V_x,2)+math.pow(self.V_y,2))
        

    def W_sub(self,msg):
        self.angular_velocity=msg.angular_velocity.z

    '''def ob_sub(self,msg):
        lidar message x,y,r'''
        
    def dwa_control(self,x, waypoint):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, waypoint)

        return u, trajectory

    def motion(self, x):
        """
        motion model
        """

        x[2] += self.yaw
        x[0] += self.cur_x
        x[1] += self.cur_y
        x[3] = self.velocity
        x[4] = self.angular_velocity
        
        return x

    def calc_dynamic_window(self,x):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.min_speed, self.max_speed,
            -self.max_yaw_rate, self.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - self.max_accel * self.dt,
            x[3] + self.max_accel * self.dt,
            x[4] - self.max_delta_yaw_rate * self.dt,
            x[4] + self.max_delta_yaw_rate * self.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    
        return dw


    def predict_trajectory(self,x_init, v, y):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= self.predict_time:
            x = self.motion(x, [v, y],self.dt)
            trajectory = np.vstack((trajectory, x))
            time += self.dt
            
        return trajectory


    def calc_control_and_trajectory(self,x, dw, waypoint):
        """
        calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, waypoint)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory

                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[0] = -self.min_speed

        return best_u, best_trajectory

    def calc_obstacle_cost(self,trajectory):
        """
        calc obstacle cost inf: collision
        """
        ox = self.ob[:, 0]
        oy = self.ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)


        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = self.ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= self.robot_length / 2
        right_check = local_ob[:, 1] <= self.robot_width / 2
        bottom_check = local_ob[:, 0] >= -self.robot_length / 2
        left_check = local_ob[:, 1] >= -self.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK

    def calc_to_goal_cost(self,trajectory,waypoint):
        """
            calc to goal cost with angle difference
        """

        dx = waypoint[0] - trajectory[-1, 0]
        dy = waypoint[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

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
                print("GOAL!")

        return waypoint, self.ex_waypoint

    def Distance_To_Goal(self,waypoint):
        if self.cur_x == 0.0 and self.cur_y == 0.0:
            return

        else:
            error_x = waypoint[0] - self.cur_x
            error_y = waypoint[1] - self.cur_y
            self.dist_to_goal = math.sqrt(error_x * error_x + error_y * error_y)

pp = Pathplanner()

def main():
    print(__file__ + " start!!")
    rospy.init_node("DWA",anonymous=True)
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    trajectory = np.array(x)

    while not rospy.is_shutdown():
        waypoint = pp.waypoint_step()[0]
        pp.ex_waypoint=pp.waypoint_step()[1]
        u, predicted_trajectory = pp.dwa_control(x, waypoint)
        x = pp.motion(x, u, pp.dt)
        trajectory = np.vstack((trajectory, x))
        dw=pp.calc_dynamic_window(x)
        best_vw=pp.calc_control_and_trajectory(x,dw,waypoint)[0]
        best_v=best_vw[0]
        best_w=best_vw[1]
        
        if best_v==0.0 and best_w==0.0:
            print("error")
            pass
        else:
            pp.v_pub.publish(best_v)
            pp.w_pub.publish(best_w)
            print("way", waypoint)
            print("v,w", best_vw)
            time.sleep(1)

    rospy.spin()

    print("Done")

if __name__ == '__main__':
     main()
