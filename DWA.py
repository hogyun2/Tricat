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

show_animation = False

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
        self.max_delta_yaw_rate =  2/35*math.pi*0.1  # [rad/ss]
        self.v_resolution = 0.1  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 1.0  # [s]
        self.to_goal_cost_gain = 0.15 #heading,alpha
        self.speed_cost_gain = 1.0 #velocity,gamma
        self.obstacle_cost_gain = 1.0 #dist,beta
        self.line_cost_gain = 1.0 #delta
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
        self.waypoints = np.array([[12.122378,-8.989864],
                                   [5,10],
                                   [10,10]])
        self.i=0
        self.tolerance=2
        self.dist_to_goal=30
        self.u_pub=rospy.Publisher('/V',Float64,queue_size=10)
        self.w_pub=rospy.Publisher('/W',Float64,queue_size=10)


    def dwa_control(self,x, waypoint, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x)

        u, trajectory = self.calc_control_and_trajectory(x, dw ,waypoint, ob)

        return u, trajectory

    def GPS_sub(self,msg):
        self.cur_x=msg.pose.pose.position.x #x position
        self.cur_y=msg.pose.pose.position.y # y position
        self.yaw = msg.pose.pose.position.z # yaw
        
    
    def V_sub(self,msg):
        self.V_x = msg.twist.twist.linear.x
        self.V_y = msg.twist.twist.linear.y
        self.velocity=np.sqrt(np.power(self.V_x,2)+np.power(self.V_y,2))
        

    def W_sub(self,msg):
        self.angular_velocity=msg.angular_velocity.z##
        

    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += self.yaw #deg_prime
        x[0] += self.cur_x #x_prime
        x[1] += self.cur_y#y_prime
        x[3] = self.velocity #velocity 
        x[4] = self.angular_velocity #yaw
        
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


    def calc_control_and_trajectory(self,x, dw, waypoint, ob):
        """
        calculation final input with dynamic window
        """
        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])
        obstacle_list=np.array([])
        for i in range(len(self.ob)):
            obs_dist = np.sqrt((np.power(self.ob[i][0] - x[0] , 2) + np.power(self.ob[i][1] - x[0] , 2)))
            if obs_dist<10:
                obstacle_list=np.append(obstacle_list,self.ob[i])
                set(obstacle_list)
            if len(obstacle_list)==0:
                k=0
            else:
                k=1


        for v in np.arange(dw[0], dw[1], self.v_resolution):
            for y in np.arange(dw[2], dw[3], self.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, waypoint)
                speed_cost = self.speed_cost_gain * (self.max_speed - trajectory[-1, 3])
                ob_cost = k * self.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob)
                final_cost = self.sigma * (to_goal_cost + speed_cost + ob_cost)

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
                        best_u[1] = -self.max_delta_yaw_rate                    
          
        #self.output_pub.publish(best_u)
        return best_u, best_trajectory

    def calc_obstacle_cost(self,trajectory, ob):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)
        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self,trajectory, waypoint):
        """
            calc to goal cost with angle difference
        """

        dx = waypoint[0] - trajectory[-1, 0]
        dy = waypoint[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))
        
        return cost

    def waypoint_step(self,x):

        if self.i + 1 != len(self.waypoints):
            if self.dist_to_goal > self.tolerance:
                waypoint = self.waypoints[self.i] #not arrived
                
            elif self.dist_to_goal <= self.tolerance:
                waypoint = self.waypoints[self.i + 1]
                self.i = self.i + 1 #arrived 
                

        elif self.i + 1 == len(self.waypoints):
            waypoint = self.waypoints[self.i] #no more waypoint
            if self.dist_to_goal <= self.tolerance:
                print("GOAL!")
        #print ("way:",waypoint)
        return waypoint

    def Distance_To_Goal(self,waypoint):
        if self.cur_x == 0.0 and self.cur_y == 0.0: #and self.psi == 0.0:
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
        return self.dist_to_goal

config = Pathplanner()

def main():
    print(__file__ + " start!!")
    rospy.init_node("DWA",anonymous=True)
    x = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0]) 
    trajectory = np.array(x)
    ob = config.ob

    while not rospy.is_shutdown():
        waypoint = config.waypoint_step(x)
        u, predicted_trajectory = config.dwa_control(x, waypoint, ob)
        x = config.motion(x, u, config.dt)
        trajectory = np.vstack((trajectory, x)) 
        config.Distance_To_Goal(waypoint)
        dw=config.calc_dynamic_window(x)
        config.predict_trajectory(x,u[0],u[1])
        config.calc_to_goal_cost(trajectory,waypoint)
        config.calc_obstacle_cost(trajectory,ob)
        best_uw=config.calc_control_and_trajectory(x,dw,waypoint,ob)[0]
        best_u=best_uw[0]
        best_w=best_uw[1]
        store=best_uw
        print("way",waypoint)

        if best_u==0.0 and best_w==0.0:
            pass
        else:
            config.u_pub.publish(best_u)
            config.w_pub.publish(best_w)
            #time.sleep(0.5)
            
        
        
    rospy.spin()

    print("Done")

if __name__ == '__main__':
     main()
