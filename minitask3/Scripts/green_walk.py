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

class Green_Walk:
    SLEEP_RATE = 10
    WALK_SPEED = 0.15
    WALK_TURN_SPEED = 0.3
    DIR_RIGHT = 270

    def __init__(self) -> None:
        self.bridge = cv_bridge.CvBridge()
        rospy.Subscriber('current_state_mt3', state_mt3, self.state_callback)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.state_pub = rospy.Publisher('current_state_mt3', state_mt3, queue_size=10)
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.init_node('Green_Walk', anonymous=True)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5
        self.state_green_walk = 0
        self.finished_action = 0
        self.image_resized = []
        self.mask = []
        self.image = []
        self.width = 0

    def callback_laser(self, msg):
        self.ranges = msg.ranges
    
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        self.mask = cv2.inRange(h, 50, 70)
        self.image = cv2.bitwise_and(self.image, self.image, mask = self.mask)
        (h, w) = self.image.shape[:2]
        self.width = w
        self.image_resized = cv2.resize(self.image, (int(w/4),int(h/4)))

    def state_callback(self, msg):
        self.state_green_walk = msg.state_green_walk
        self.finished_action = msg.finished_action

    def green_dist(self):
        if(np.size(self.image_resized) == 0): return
        # calculate moments of binary image
        M = cv2.moments(self.mask)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / (M["m00"] + 1))
        cY = int(M["m01"] / (M["m00"] + 1))

        centre_mass = (cX, cY)
        return centre_mass
    
    def lowest_average_reading(self, centre, m):
        points = sorted(self.ranges[(centre - m) % 360:centre - 1:] + self.ranges[centre % 360: (centre + m) % 360:])
        points = [x if x < self.range_max else 3.5 for x in points]
        if len(points):
            return sum(points[:3:]) / 3
        
    def obstacle_condition(self, conditions):
        detection_dist = [self.lowest_average_reading(x[0], 10) for x in conditions if self.lowest_average_reading(x[0], 10) < x[1]]
        # print(detection_dist)
        return len(detection_dist)
    
    def wall_dist(self):
        points = self.ranges[self.DIR_RIGHT:self.DIR_RIGHT+20]
        if len(points):
            return min(points)
    
    def run(self):
        vel_msg = Twist()
        r = rospy.Rate(10)
        cond = [(0, 0.5), (315, 0.45), (335, 0.45), (25, 0.45), (45, 0.45), (90, 0.45)]
        while not rospy.is_shutdown():
            if self.state_green_walk and not self.finished_action:
                print("Entering green walk")
                # Move around obstacle towards green, else move towards green
                vel_msg.linear.x = self.WALK_SPEED
                if not self.obstacle_condition(cond):
                    while not self.obstacle_condition(cond) and not (self.green_dist() == (0,0)):
                        dist = self.green_dist()
                        if not dist or dist == (0,0): continue
                        error = dist[0] - int(self.width/2)
                        kp = 0.0002
                        P = -error
                        vel_msg.angular.z = kp*P
                        self.pos_pub.publish(vel_msg)
                        r.sleep()
                else:
                    vel_msg.linear.x = self.WALK_SPEED / 2
                    while self.obstacle_condition(cond) and not (self.green_dist() == (0,0)):
                        dist = self.wall_dist()
                        if not dist or dist == (0,0): continue
                        error = dist - 0.45
                        kp = 0.2
                        P = -error
                        vel_msg.angular.z = kp*P
                        self.pos_pub.publish(vel_msg)
                        r.sleep()

                self.pos_pub.publish(Twist())
                print("Exiting green walk")
                self.state_pub.publish(state_mt3(finished_action = 1))
            r.sleep()

if __name__ == '__main__':
    try:
        node = Green_Walk()
        node.run()
    except rospy.ROSInterruptException:
        pass