import rospy
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap, SetMap
from sensor_msgs.msg import LaserScan
from minitask5.msg import frontiers
import tf
import numpy as np
import math

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class frontier_finder():

    def __init__(self):
        self.corr_x = 0.3
        self.grid = []
        self.resolution = 0.1
        self.width = 194
        self.height = 194
        self.origin = Pose()
        self.pub_frontier = rospy.Publisher('/frontiers', frontiers, queue_size=10)
        self.sub_occupancy = rospy.Subscriber('/map_frontier', OccupancyGrid, self.callback_occ)
        rospy.init_node('frontier_finder', anonymous=True)
        pass

    def callback_occ(self, msg: OccupancyGrid):
        self.grid = msg.data
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin

    def to_grid(self, px, py, origin_, resolution):
        offsetx = (1/resolution)*(px - origin_[1] + self.corr_x)
        offsety = (1/resolution)*(py - origin_[0])
        return (int(offsetx), int(offsety))
        
    def to_world(self, gx, gy, origin, resolution):
        offsetx = (resolution)*gx + origin[1] - self.corr_x
        offsety = (resolution)*gy + origin[0]
        return (offsetx, offsety)

    def to_index(self, gx, gy, size_x):
        return gx * size_x + gy
    
    def run():

        # PROCESS LINES IN OCCUPENCY GRID
        # DETERMINE FRONTIERS
        # PUBLISH LIST OF FRONTIERS
        
        pass

if __name__ == '__main__':
    try:
        rospy.loginfo("You have reached the destination")
        frontier_ = frontier_finder()
        frontier_.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")