#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from obstacle_avoidance.srv import GetDistance, GetDistanceResponse
import math

class GetDistanceNode:
    
    def __init__(self) -> None:
        
        rospy.init_node("get_distance_node" , anonymous=True)
        
        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.get_pos)
        self.service=rospy.Service("get_distance",GetDistance,self.calc_dist)
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
        self.pose_x=0
        self.pose_y=0
        

    def get_pos(self,odom):
        self.pose_x = odom.pose.pose.position.x
        self.pose_y = odom.pose.pose.position.y
        
    def calc_dist(self,req):
        obs_pos = self.obstacles[req.obstacle_name]
        distance = math.sqrt(((self.pose_x - obs_pos[0]) ** 2) + ((self.pose_y - obs_pos[1]) ** 2))
        res = GetDistanceResponse()
        res.distance = distance
        return res


if __name__ == "__main__":
    gd = GetDistanceNode()
    rospy.spin()