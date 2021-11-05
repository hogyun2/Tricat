#! /usr/bin/env python2.7

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

data=Imu()

class Pathplanner():
    """
    simulation parameter class
    """

    def __init__(self):
        rospy.Subscriber('/pose',Odometry,self.GPS_sub)
        rospy.Subscriber('/gps_data/fix_velocity',TwistWithCovarianceStamped,self.V_sub)
        rospy.Subscriber('/imu/data',Imu,self.W_sub)
        # robot parameter
        self.robot_stuck_flag_cons =0.001
        self.max_speed = 1.8  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 2/35*math.pi  # [rad/s]
        self.max_accel = 1.0  # [m/ss]
        self.max_delta_yaw_rate =  2/35*math.pi  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.alpha = 1.0 #heading,alpha
        self.gamma = 1.0 #velocity,gamma
        self.beta = 10.0 #dist,beta
        self.delta = 10.0 #delta
        self.d_max = 1000
        self.sigma = 1
        self.cur_x=0.0
        self.cur_y=0.0
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


    def dwa_control(self,x, waypoint):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw, waypoint)

        return u, trajectory

    def GPS_sub(self,msg):
        self.cur_x=msg.pose.pose.position.x 
        self.cur_y=msg.pose.pose.position.y
        self.yaw = msg.pose.pose.position.z
        
    
    def V_sub(self,msg):
        self.V_x = msg.twist.twist.linear.x
        self.V_y = msg.twist.twist.linear.y
        self.velocity = math.sqrt(math.pow(self.V_x,2)+math.pow(self.V_y,2))
        

    def W_sub(self,msg):
        self.angular_velocity=msg.angular_velocity.z
        

    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]
        
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
        max_cost = 0.0
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])
        h=[0.0,0.0]
        h_cost=0.0
        d=[0.0,0.0]
        d_cost=0.0
        v_max=0.0
        v_min=0.0
        v_cost=0.0
        l=[0.0,0.0]
        l_cost=0.0

        if len(self.ob)==0:
            k=0
        else:
            k=1


        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)

                if self.heading_cost(trajectory, waypoint)<h[0]:
                    h[0]=self.heading_cost(trajectory, waypoint)
                elif self.heading_cost(trajectory, waypoint)>h[1]:
                    h[1]=self.heading_cost(trajectory, waypoint)
                h_cost=(self.heading_cost(trajectory, waypoint)-h[0])/(h[1]-h[0])

                if self.dist_cost(trajectory)<d[0]:
                    d[0]=self.dist_cost(trajectory)
                elif self.dist_cost(trajectory)>d[1]:
                    d[1]=self.dist_cost(trajectory)
                d_cost=(self.dist_cost(trajectory)-d[0])/(d[1]-d[0])

                if self.velocity < v_min:
                    v_min=self.velocity
                elif self.velocity>v_max:
                    v_max=self.velocity

                if v_max-v_min==0:
                    v_cost=self.velocity
                else:
                    v_cost=(self.velocity-v_min)/(v_max-v_min)

                if self.line_cost(trajectory,waypoint)<l[0]:
                    l[0]=self.line_cost(trajectory,waypoint)
                elif self.line_cost(trajectory,waypoint)>l[1]:
                    l[1]=self.line_cost(trajectory,waypoint)
                l_cost=(self.line_cost(trajectory, waypoint)-l[0])/(l[1]-l[0])
    
        
                Heading = self.alpha * h_cost #
                Dist = k * self.beta * d_cost
                Velocity = self.gamma * v_cost# this or current v
                d_line = abs(k-1) * self.delta * l_cost

                final_cost = self.sigma * (Heading + Dist + Velocity + d_line)

                # search minimum trajectory

                if max_cost <= final_cost:
                    max_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < self.robot_stuck_flag_cons \
                            and abs(x[3]) < self.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[0] = self.min_speed
                print("G",final_cost)
                time.sleep(1)

        return best_u, best_trajectory

    def line_cost(self,trajectory,waypoint):
        x_prime=trajectory[-1,0]
        y_prime=trajectory[-1,1]
        yb=waypoint[1]
        ya=self.ex_waypoint[1]
        xb=waypoint[0]
        xa=self.ex_waypoint[0]
        dl = x_prime*(yb-ya)-y_prime*(xb-xa)-xa*(yb-ya)+ya*(xb-xa)/np.sqrt(np.power((yb-ya),2)+np.power((xb-xa),2))
        d_line=self.d_max-dl

        return d_line

    def dist_cost(self,trajectory):
        ox = self.ob[0,0]
        oy = self.ob[0,1] #chldntjs ghlvl wkddoanfdml dnlcl
        dx = trajectory[-1, 0] - ox
        dy = trajectory[-1, 1] - oy
        ob_dist = math.sqrt(math.pow(dx,2)+math.pow(dy,2))
        return ob_dist


    def heading_cost(self,trajectory, waypoint):
        dx = waypoint[0] - trajectory[-1, 0]
        dy = waypoint[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        heading = error_angle - trajectory[-1, 2]
        
        return heading

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

            theta = math.atan2(error_x, error_y) * 180 / math.pi #y axis = north
            
            error_N = theta - self.yaw  
            
            if 180 <= error_N and error_N < 360:
                error_N = error_N - 360
            elif -360 < error_N and error_N < -180:
                error_N = error_N + 360

            self.dist_to_goal = math.sqrt(error_x * error_x + error_y * error_y)
            # error_Distance : the distance left to waypoint
            #error_N = round(error_N * 2, -1) / 2
        #print (self.dist_to_goal)

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