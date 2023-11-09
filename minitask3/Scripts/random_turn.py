import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from minitask3.msg import state_mt3
import numpy as np
import random
import cv2, cv_bridge
import queue
import tf

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class Random_Turn:
    SLEEP_RATE = 10
    WALK_TURN_SPEED = 0.3
    
    def __init__(self) -> None:
        self.state_pub = rospy.Publisher('current_state_mt3', state_mt3, queue_size=10)
        rospy.init_node('Random_Turn', anonymous=True)
        rospy.Subscriber('current_state_mt3', state_mt3, self.state_callback)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5
        self.state_random_turn = 0
        self.finished_action = 0

    def callback_laser(self, msg):
        self.ranges = msg.ranges

    def state_callback(self, msg):
        self.state_random_turn = msg.state_random_turn
        self.finished_action = msg.finished_action

    def lowest_average_reading(self, centre, m):
        points = sorted(self.ranges[(centre - m) % 360:centre - 1:] + self.ranges[centre % 360: (centre + m) % 360:])
        points = [x if x < self.range_max else 3.5 for x in points]
        if len(points):
            return sum(points[:3:]) / 3
        
    def obstacle_condition(self, conditions):
        detection_dist = [self.lowest_average_reading(x[0], 10) for x in conditions if self.lowest_average_reading(x[0], 10) < x[1]]
        # print(detection_dist)
        return len(detection_dist)
    
    def run(self):
        vel_turn = Twist()
        r = rospy.Rate(10)
        cond = [(0, 0.5), (315, 0.45), (335, 0.45), (25, 0.45), (45, 0.45), (90, 0.45)]
        while not rospy.is_shutdown():
            if self.state_random_turn and not self.finished_action:
                print("Entering random turn")
                t = rospy.Time.now().to_sec()
                r_t = random.randint(3, 10)
                vel_turn.angular.z = self.WALK_TURN_SPEED if random.randrange(0, 2, 1) else -self.WALK_TURN_SPEED
                while rospy.Time.now().to_sec() - t < rospy.Duration(r_t).to_sec() or self.obstacle_condition(cond):
                    self.pos_pub.publish(vel_turn)
                    r.sleep()
                self.pos_pub.publish(Twist())
                print("Exiting random turn")
                self.state_pub.publish(state_mt3(finished_action = 1))
            r.sleep()

if __name__ == '__main__':
    try:
        node = Random_Turn()
        node.run()
    except rospy.ROSInterruptException:
        pass