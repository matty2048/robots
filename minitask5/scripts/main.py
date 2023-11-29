import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan, Image
from minitask5.msg import controller
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
        self.states = controller()
        self.pos = Position(0, 0, 0)

        # Publisher and Subcriber initialisation
        self.control_pub = rospy.Publisher('mt5_control', controller, queue_size=10)
        self.state_sub = rospy.Subscriber('mt5_control', controller, self.callback_controller)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)

        # Create main node
        rospy.init_node('main_controller', anonymous=False)

    def callback_laser(self, msg):
        self.ranges = msg.ranges

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    def callback_controller(self, msg):
        self.states.state_find_goal = msg.state_find_goal
        self.states.state_object_avoid = msg.state_object_avoid
        self.states.state_random_walk = msg.state_random_walk

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        # Whilst not shutdown OR Found all objects
        while not rospy.is_shutdown():
            
            self.control_pub.publish(controller(state_object_avoid = 1))
            # If no green objects in sight
                # Random Walk
            # elif 
            r.sleep()
            

if __name__ == '__main__':
    try:
        main = Main()
        main.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass