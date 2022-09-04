#!/usr/bin/python3

import rospy
from nav_msgs.msg import Odometry
from obstacle_avoidance.msg import Obst
import math
from obstacle_avoidance.srv import GetDistance,GetDistanceResponse


class ClosestObstacle:
    
    def __init__(self) -> None:
        
        rospy.init_node("closest_obstacle_node" , anonymous=True)
        # rospy.loginfo("waiting for get distance")
        # rospy.wait_for_service('get_distance')
        
        # self.getDistService = rospy.ServiceProxy('get_distance', GetDistance)

        self.odom_subscriber = rospy.Subscriber("/odom" , Odometry , callback=self.read_distance)
        self.obstacle_publisher = rospy.Publisher('/ClosestObstacle' , Obst , queue_size=10)
        
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
        

    def read_distance(self,odom):
        pose_x = odom.pose.pose.position.x
        pose_y = odom.pose.pose.position.y
        
        min_distance = 100000
        for obstacle,obs_pos in self.obstacles.items():    
            distance = math.sqrt(((pose_x - obs_pos[0]) ** 2) + ((pose_y - obs_pos[1]) ** 2))
            if distance < min_distance:
                closest = obstacle
                min_distance = distance
        obs=Obst()
        obs.obstacle_name=closest
        obs.distance=min_distance
        # rospy.loginfo(f"closest: {closest} {min_distance}")
        self.obstacle_publisher.publish(obs)

    # #with service
    # def read_distance(self):
    #     min_distance = 100000
    #     for obstacle in self.obstacles.keys():    
    #         res = self.getDistService(obstacle)
    #         distance = res.distance
    #         if distance < min_distance:
    #             closest = obstacle
    #             min_distance = distance
    #     obs=Obs()
    #     obs.obstacle_name=closest
    #     obs.distance=min_distance
    #     rospy.loginfo(f"closest: {closest} {min_distance}")
    #     self.obstacle_publisher.publish(obs)

    

if __name__ == "__main__":
    co = ClosestObstacle()
    # while True:
    #     co.read_distance()
    rospy.spin()