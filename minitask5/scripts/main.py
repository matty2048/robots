import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan, Image
from minitask5.msg import image_proc
import tf
import numpy as np
import math

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class Main:
    SLEEP_RATE = 10

    def __init__(self):
        # Parameter initialisation
        self.param_max_green_boxes = rospy.get_param('/main/max_green_boxes')
        self.param_max_red_hydrants = rospy.get_param('/main/max_red_hydrants')

        # Variable initialisation
        self.objects_found = []

        # Publisher and Subcriber initialisation
        # rospy.Subscriber('scan', LaserScan, self.callback_laser)
        # rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('/image_proc', image_proc, self.callback_objects)

        # Create main node
        rospy.init_node('main', anonymous=False)

    def callback_objects(self, msg):
        pass

    # def callback_laser(self, msg):
    #     self.ranges = msg.ranges

    # def callback_odom(self, msg):
    #     quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
    #     (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
    #     self.pos.theta = yaw
    #     self.pos.x = msg.pose.pose.position.x
    #     self.pos.y = msg.pose.pose.position.y

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        # Whilst not shutdown OR Found all objects
        while not rospy.is_shutdown():
            
            r.sleep()
            

if __name__ == '__main__':
    try:
        main = Main()
        main.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass