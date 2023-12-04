import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from sensor_msgs.msg import LaserScan, Image
from minitask5.msg import frontiers
import tf
import numpy as np
import math

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class move_to:
    SLEEP_RATE = 2

    def __init__(self):
        # Parameter initialisation
        self.param_max_green_boxes = rospy.get_param('/main/max_green_boxes')
        self.param_max_red_hydrants = rospy.get_param('/main/max_red_hydrants')
        self.frontiers = []
        # Variable initialisation
        self.pos = Position(0, 0, 0)

        # Turn condition variable initialisation
        self.turn = 0
        self.reverse = 0

        # Publisher and Subcriber initialisation
        # rospy.Subscriber('/frontiers', frontiers, self.callback_frontiers)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)

        # Create main node
        rospy.init_node('move_to.py', anonymous=False)

    def callback_frontiers(self):
        pass

    def callback_laser(self, msg):
        self.ranges = msg.ranges
        front = msg.ranges[340:360] + msg.ranges[0:20]
        back = msg.ranges[160:200]
        if min(back) > 0.5 and min(front) < 0.4:
            self.reverse = 1
        elif min(front) < 0.5:
            self.turn = 1
        else:
            self.turn = 0
            self.reverse = 0

    def moveToGoal(self,xGoal,yGoal):

        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        # Whilst not shutdown OR Found all objects
        while not rospy.is_shutdown():
            # self.moveToGoal(0, -1)
            r.sleep()

if __name__ == '__main__':
    try:
        main = move_to()
        main.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass