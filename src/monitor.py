#!/usr/bin/python3
import rospy
from obstacle_avoidance.msg import Obst

class ObsMonitor:
    
    def __init__(self) -> None:
        
        rospy.init_node("monitor" , anonymous=False)
        self.odom_subscriber = rospy.Subscriber("/ClosestObstacle" , Obst , callback=self.Obs_callback)

        
        
    def Obs_callback(self, msg : Obst):
        rospy.loginfo(msg.obstacle_name)
        # rospy.loginfo(msg.distance)
        
        
if __name__ == "__main__":
    obsmonitor = ObsMonitor()
    
    rospy.spin()