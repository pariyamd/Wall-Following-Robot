#!/usr/bin/python3

import re
from rospkg import RosPack
import rospy
from nav_msgs.msg import Odometry
from obstacle_avoidance.msg import Obst
from math import radians,pi
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
from geometry_msgs.msg import Twist


class AvoidObstacleNode:
    
    def __init__(self) -> None:
        
        rospy.init_node("avoid_obstacle_node" , anonymous=True)
        # rospy.wait_for_service('closest_obstacle_node')
        
        # self.obstacle_publisher = rospy.Subscriber('/ClosestObstacle' , Obst ,callback=self.check_distance)
        self.cmd_publisher = rospy.Publisher('/cmd_vel' , Twist , queue_size=10)
        
        self.obstacles={
            "bookshelf" : (2.64, -1.55),
            "dumpster" :  (1.23, -4.57),
            "barrel" :    (-2.51, -3.08),
            "postbox" :   (-4.47, -0.57),
            "brick_box"	: (-3.44, 2.75),
            "cabinet" :	  (-0.45, 4.05),
            "cafe_table": (1.91, 3.37),
            "fountain" :  (4.08, 1.14)
        }
        self.angular_speed=0.1
    
    def angular_error(self, diff):
        if diff < -pi:
            return diff+ 2 * pi
        if diff > pi:
            return diff - 2 * pi
        return diff
        
    def get_heading(self):
        msg = rospy.wait_for_message("/odom" , Odometry)
        orientation = msg.pose.pose.orientation
        self.pose_x= msg.pose.pose.position.x
        self.pose_y= msg.pose.pose.position.y
        roll, pitch, yaw = tf.transformations.euler_from_quaternion((
            orientation.x ,orientation.y ,orientation.z ,orientation.w
        )) 
        self.yaw = yaw
        return yaw

    def rotate(self,remaining):
        rospy.loginfo(f"rotating {remaining} radians")
        prev_angle = self.get_heading()
        twist = Twist()
        twist.angular.z = self.angular_speed
        self.cmd_publisher.publish(twist)
        
        # rotation loop
        while remaining >= 0.01:
            current_angle = self.get_heading()  
            delta = abs(prev_angle - current_angle)
            delta=self.angular_error(delta)
            remaining -= delta
            prev_angle = current_angle
            print(remaining)

        self.cmd_publisher.publish(Twist())
        rospy.sleep(1)

    def check_distance(self):
        while True:
            obs= rospy.wait_for_message('/ClosestObstacle' , Obst )
            rospy.loginfo(obs.distance)
            if obs.distance<1.5:
                rospy.loginfo("got too close!")
                ##stop
                twist = Twist()
                twist.linear.x = 0
                self.cmd_publisher.publish(twist)
                # self.cmd_publisher.publish(Twist())
                rospy.sleep(1)
                msg = rospy.wait_for_message("/scan" , LaserScan, timeout=1)
                ranges=np.array(msg.ranges)
                min_ind=np.argmin(ranges)
                
                rospy.loginfo(min_ind)
                rospy.loginfo(180-min_ind)
                
                remaining = self.angular_error(radians(180-min_ind))
                self.rotate(remaining)


if __name__ == "__main__":
    co = AvoidObstacleNode()
    co.check_distance()
    # rospy.spin()