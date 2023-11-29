import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np
import math

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class Main:
    SLEEP_RATE = 5
    def __init__(self):
        pass

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        r.sleep()
        while not rospy.is_shutdown():
            pass

if __name__ == '__main__':
    try:
        main = Main()
        main.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass