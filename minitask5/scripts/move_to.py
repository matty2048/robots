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
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header


class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class move_to:
    SLEEP_RATE = 5

    def __init__(self):
        # Parameter initialisation
        self.param_max_green_boxes = rospy.get_param('/main/max_green_boxes')
        self.param_max_red_hydrants = rospy.get_param('/main/max_red_hydrants')
        
        # Variable initialisation
        self.pos = Position(0, 0, 0)
        self.frontiers = []
        self.corr_x = 0.3

        # Turn condition variable initialisation
        # self.turn = 0
        self.reverse = 0
        self.turn = 0
        self.quat = 0
        # Publisher and Subcriber initialisation
        rospy.Subscriber('/frontiers', frontiers, self.callback_frontiers)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.stamp_pub = rospy.Publisher('point', PointStamped, queue_size=10)
        self.last_goal = 0
        # Create main node
        rospy.init_node('move_to.py', anonymous=False)

    def callback_frontiers(self, msg):
        self.frontiers = msg.frontiers

    def callback_laser(self, msg):
        self.ranges = msg.ranges
        front = msg.ranges[330:360] + msg.ranges[0:30]
        front_turn = msg.ranges[320:360] + msg.ranges[0:40]
        back = msg.ranges[160:200]
        # if currently turning then keep turning until
        # if self.turn and min(front_turn) < 0.4:
        #     self.turn = 1
        # # elif reversing keep reversing until
        # elif self.reverse and min(back) < 0.5:
        #     self.reverse = 1
        # else: 
        #     if min(back) > 0.5 and min(front) < 0.2:
        #         self.reverse = 1
        #         self.turn = 0
        #     elif min(front) < 0.2:
        #         self.turn = 1
        #         self.reverse = 0
        #     else:
        #         self.turn = 0
        #         self.reverse = 0

    def moveToGoal(self,xGoal,yGoal,orientation = 1.0):

        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0)) and not rospy.is_shutdown()):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()
        if((xGoal, yGoal) == self.last_goal):
            return
        self.last_goal = (xGoal, yGoal)
        stamp = PointStamped(header=Header(stamp=rospy.Time.now(),
                                              frame_id="global"),
                                point=Point(xGoal, yGoal, 0.0))
        
        self.stamp_pub.publish(stamp)
        
        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = orientation

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(30))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False

    def callback_odom(self, msg):
        self.quat = msg.pose.pose.orientation
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    def decide_goal(self):
        fronts = self.frontiers
        #print(fronts)
        if not len(fronts): return None
        distances = [math.dist((self.pos.x, self.pos.y), (i.x, i.y)) for i in fronts]
        goals = sorted(list(zip(distances, fronts)), key= lambda x: x[0])
        goal = goals[0][1]
        goal.y = goal.y
        return goal
        # for goal in goals:
        #     if goal[0] < 1: continue
        #     else: return goal[1]
        # return goals[0][1]
        

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        # Whilst not shutdown OR Found all objects
        r.sleep()
        t = rospy.Time.now().to_sec()
        num_secs = 2
        back = self.ranges[160:200]
        front = self.ranges[340:360] + self.ranges[0:20]
        while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec() and (min(back) > 0.4 and min(front) < 0.4):
            vel_msg = Twist()
            vel_msg.linear.x = -0.04
            self.vel_pub.publish( vel_msg )
            r.sleep()

        while not rospy.is_shutdown():
            
            goal: Point = self.decide_goal()
            if goal == None: continue
            if not self.moveToGoal(goal.x, goal.y):
                #goal = np.random.shuffle(self.frontiers)[0]
                print("failed to reach goal")

            # Move to main controller set up 
            # Check for certainty of position with covariance to activate move_base
            r.sleep()

if __name__ == '__main__':
    try:
        main = move_to()
        main.run()
    except rospy.ROSInterruptException:
        pass