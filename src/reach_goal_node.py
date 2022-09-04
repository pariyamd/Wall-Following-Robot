#!/usr/bin/python3

import math
from turtle import distance

from numpy import sign
import tf
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from nav_msgs.msg import Odometry

regions = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
    'all': None
}
state_dict = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'follow point',
}

class PIDController():


    def __init__(self):
        
        rospy.init_node('reach_goal_node', anonymous=False)
        self.state = 0
        self.goal_point = (3, -1)
        self.odom_msg = None
        
        self.dt = 0.005
        self.D = 0.75
        rate = 1/self.dt
        
        self.r = rospy.Rate(rate)
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=5)

    def get_distance_from_wall(self, dir):
        msg = rospy.wait_for_message("/scan" , LaserScan)
        ranges = np.array(msg.ranges)
        if len(ranges) == 0:
            return

        global regions
        regions = {
            'right':  min(min(ranges[245:289]), 10),
            'fright': min(min(ranges[290:336]), 10),
            'front':  min(min(ranges[:23]), min(ranges[337:]), 10),
            'fleft':  min(min(ranges[24:70]), 10),
            'left':   min(min(ranges[71:115]), 10),
            'all': [dis for dis in ranges]
        }

        self.check_route()
        

    def change_state(self, state):
        if state != self.state:
            self.state = state
            rospy.loginfo(f"current state: {state_dict[state]}")


    def follow_wall(self):
        msg = Twist()
        msg.linear.x = 0.15
        return msg


    def find_wall(self):
        msg = Twist()
        msg.linear.x = 0.1
        msg.angular.z = -0.5
        return msg

        
    def turn_left(self):
        msg = Twist()
        msg.angular.z = 0.5
        return msg


    def get_odom_msg(self):
        self.odom_msg = rospy.wait_for_message("/odom", Odometry)

    
    def get_current_z_angle(self):
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            self.odom_msg.pose.pose.orientation.x,
            self.odom_msg.pose.pose.orientation.y,
            self.odom_msg.pose.pose.orientation.z,
            self.odom_msg.pose.pose.orientation.w
        ))
        return yaw    

    
    def calculate_angle_to_point(self):
        self.get_odom_msg()
        point_x, point_y = self.goal_point
        angle_to_point = math.atan2(point_y - self.odom_msg.pose.pose.position.y, point_x - self.odom_msg.pose.pose.position.x)
        
        angle_to_point = (2*math.pi + angle_to_point) % (2*math.pi)
        return angle_to_point
        
        
    def calculate_angle_difference(self):
        self.get_odom_msg()
        yaw = self.get_current_z_angle()
        
        angle_to_point = self.calculate_angle_to_point()
        yaw = (2*math.pi + yaw) % (2*math.pi)

        diff_angle = angle_to_point - yaw
        return diff_angle
                

 
    def follow_point(self):
        diff_angle = self.calculate_angle_difference()
        
        msg = Twist()
        if math.degrees(diff_angle) > 1:
            msg.linear.x = 0.0
            msg.angular.z = sign(diff_angle) * 0.2
        else:
            msg.linear.x = 0.15
            msg.angular.z = sign(diff_angle) * 0.1
            
        return msg



    def check_route(self):
        global regions
        rgns = regions
        
        state_description = ''
        d = self.D
        
        degree = int(math.degrees(self.calculate_angle_difference()))
        range_index = degree
        range_begin = (range_index - 10) % len(rgns['all'])
        range_end = (range_index + 10) % len(rgns['all'])
        flag = False
        for i in range(range_begin, range_end):
            if rgns['all'][i] <= d:
                flag = True
            
        
        
        # if rgns['front'] > d and rgns['fleft'] > d and rgns['fright'] > d:
        if not flag:
            state_description = 'case 1 - follow point'
            self.change_state(3)
        elif rgns['front'] > d and rgns['fleft'] > d and rgns['fright'] > d:
            state_description = 'case 1 - nothing'
            self.change_state(0)
        elif rgns['front'] < d and rgns['fleft'] > d and rgns['fright'] > d:
            state_description = 'case 2 - front'
            self.change_state(1)
        elif rgns['front'] > d and rgns['fleft'] > d and rgns['fright'] < d:
            state_description = 'case 3 - fright'
            self.change_state(2)
        elif rgns['front'] > d and rgns['fleft'] < d and rgns['fright'] > d:
            state_description = 'case 4 - fleft'
            self.change_state(0)
        elif rgns['front'] < d and rgns['fleft'] > d and rgns['fright'] < d:
            state_description = 'case 5 - front and fright'
            self.change_state(1)
        elif rgns['front'] < d and rgns['fleft'] < d and rgns['fright'] > d:
            state_description = 'case 6 - front and fleft'
            self.change_state(1)
        elif rgns['front'] < d and rgns['fleft'] < d and rgns['fright'] < d:
            state_description = 'case 7 - front and fleft and fright'
            self.change_state(1)
        elif rgns['front'] > d and rgns['fleft'] < d and rgns['fright'] < d:
            state_description = 'case 8 - fleft and fright'
            self.change_state(0)
        else:
            state_description = 'unknown case'
            rospy.loginfo(rgns)
        rospy.loginfo(state_description)

    def run(self):
        while not rospy.is_shutdown():
            self.get_distance_from_wall("right")
            if self.state == 0:
                msg = self.find_wall()
            elif self.state == 1:
                msg = self.turn_left()
            elif self.state == 2:
                msg = self.follow_wall()
            elif self.state == 3:
                msg = self.follow_point()
                
            self.cmd_vel.publish(msg)
            self.r.sleep()
try:
    pidc = PIDController()
    pidc.run()
except rospy.ROSInterruptException:
    rospy.loginfo("Navigation terminated.")