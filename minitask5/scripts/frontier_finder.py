import rospy
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap, SetMap
from scripts.image_proc import Point
from sensor_msgs.msg import LaserScan
from minitask5.msg import frontiers
import tf
import numpy as np
import math
import cv2
import matplotlib.pyplot as plt
 

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
        self.frontier = []
        self.pub_frontier = rospy.Publisher('/frontiers', frontiers, queue_size=10)
        self.sub_occupancy = rospy.Subscriber('/map_frontier', OccupancyGrid, self.callback_occ)
        rospy.init_node('frontier_finder', anonymous=True)
        self.plot = 0
        self.front_points = []
        pass

    def callback_occ(self, msg: OccupancyGrid):
        self.grid = msg.data
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = msg.info.origin
        self.frontier = np.zeros((self.width, self.height,1), dtype=np.uint8)
        self.find_borders()
        #self.front_points = self.connected_components()
        #print(self.front_points)

    def to_grid(self, px, py, origin_, resolution):
        offsetx = (1/resolution)*(px - origin_[1] + self.corr_x)
        offsety = (1/resolution)*(py - origin_[0])
        return (int(offsetx), int(offsety))
        
    def to_world(self, gx, gy, origin, resolution):
        offsetx = (resolution)*gx + origin[1]
        offsety = (resolution)*gy + origin[0]
        return (offsetx, offsety)

    def to_index(self, gx, gy, size_x):
        return gx * size_x + gy
    
    def find_borders(self):
        # This turns the map -> binary frontier 
        for j in range(self.height):
            for i in range(self.width):
                # loop over every node in grid
                #idx of this node
                idx = self.to_index(j,i,self.width)
                # all the neighbours of this node idx 
                neighbour_idx = [self.to_index(j, i+1, self.width), self.to_index(j, i-1, self.width), self.to_index(j+1,i, self.width), self.to_index(j-1,i, self.width)]
                status = 0
                for neighbour in neighbour_idx:
                    if (self.grid[idx] == 0):
                        if(neighbour > 0 and neighbour < len(self.grid) and self.grid[neighbour] == -1):
                            # if any neighbours are unknown then the node is a frontier
                            status = 255
                self.frontier[i][j][0] = status
        self.front_points = self.connected_components()
        outputmsg = [] 
        for point in self.front_points:
            p = Point(point[0], point[1],0,0)
            outputmsg.append(p)
        self.pub_frontier.publish(outputmsg)

    def connected_components(self):
        analysis = cv2.connectedComponentsWithStats(self.frontier, 
                                            4, 
                                            cv2.CV_32S) 
        (totalLabels, label_ids, values, centroid) = analysis
        return centroid

    def run(self):

        # PROCESS LINES IN OCCUPENCY GRID
        # DETERMINE FRONTIERS
        # PUBLISH LIST OF FRONTIERS
        r = rospy.Rate(1)
        while not rospy.is_shutdown:
            r.sleep()
        pass

if __name__ == '__main__':
    try:
        rospy.loginfo("You have reached the destination")
        frontier_ = frontier_finder()
        frontier_.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")