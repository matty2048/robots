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

class Follower:
    SLEEP_RATE = 5

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        
        self.state_pub = rospy.Publisher('current_state_mt3', state_mt3, queue_size=10)
        rospy.init_node('Follower', anonymous=True)
        
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('current_state_mt3', state_mt3, self.callback_state)
        self.finished_action = 1
        self.pos = Position(0, 0, 0)
        self.image_resized = []
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5
        self.mask = []
        self.image = []
        self.width = 0
        cv2.startWindowThread()
    
    def green_dist(self):
        if(np.size(self.image_resized) == 0): return
        # calculate moments of binary image
        M = cv2.moments(self.mask)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / (M["m00"] + 1))
        cY = int(M["m01"] / (M["m00"] + 1))

        centre_mass = (cX, cY)
        cv2.circle(self.image, centre_mass, 10, (0,0,255), 2)
        return centre_mass
    
    def callback_state(self, msg):
        self.finished_action = msg.finished_action

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        self.mask = cv2.inRange(h, 50, 70)
        self.image = cv2.bitwise_and(self.image, self.image, mask = self.mask)
        (h, w) = self.image.shape[:2]
        self.width = w
        self.image_resized = cv2.resize(self.image, (int(w/4),int(h/4)))
        cv2.imshow("original", self.image_resized)

    def callback_laser(self, msg):
        self.ranges = msg.ranges

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    # def green_location(self):
    #     th1 = cv2.inRange(self.image_resized, (36, 25, 25), (70, 255,255))
    #     mass_x, mass_y = np.where(th1 >= 200)
    #     # mass_x and mass_y are the list of x indices and y indices of mass pixels
    #     cent_x = np.average(mass_x)
    #     cent_y = np.average(mass_y)
    #     return (cent_x, cent_y)
    
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
        r = rospy.Rate(self.SLEEP_RATE)
        r.sleep()
        cond = [(0, 0.5), (315, 0.45), (335, 0.45), (25, 0.45), (45, 0.45), (90, 0.45)]
        while not rospy.is_shutdown():

            # if self.obstacle_condition(cond):
            #     while self.obstacle_condition(cond):
            #         self.pos_pub.publish(vel_turn)
            #         r.sleep()
            
            # # Walk towards green
            # previous = 0
            # while not self.obstacle_condition(cond) and (self.green_dist() == (0,0)):
            #     dist = self.green_dist()
            #     if not dist or dist == (0,0): continue
            #     error = dist[0] - int(self.width/2)
            #     print(dist, error)
            #     kp = 0.0002
            #     kd = 0.000
            #     ki = 0.000
            #     P = -error
            #     I = ki * (P - previous)
            #     previous = P
            #     sums =0   
            #     for i in past_vals.queue:
            #         sums += i
            #     D = sums / len(past_vals.queue) if sums != 0 else 0
            #     if past_vals.full(): 
            #         past_vals.get()
            #     past_vals.put(P)
            #     vel_turn.angular.z = kp*P + kd*D + I
            #     self.pos_pub.publish(vel_turn)
            #     r.sleep()
            
            # # walk forward
            # while not self.obstacle_condition(cond):
            #     self.pos_pub.publish(vel_msg1)
            #     if self.obstacle_condition(cond):
            #         self.pos_pub.publish(Twist())
            #         break
            #     r.sleep()

            # # random turn for set time or if there is an obstacle
            # t = rospy.Time.now().to_sec()
            # r_t = random.randint(3, 10)
            # while rospy.Time.now().to_sec() - t < rospy.Duration(r_t).to_sec() or self.obstacle_condition(cond):
            #     self.pos_pub.publish(vel_turn)

            #     r.sleep()
            
            # self.pos_pub.publish(Twist())

            # green detected
            r.sleep()

            if not self.finished_action:
                continue

            print("Deciding what to do")
            if self.obstacle_condition(cond):
                print("Decided to turn")
                self.state_pub.publish(state_mt3(0, 0, 1, 0))
            else:
                print("Decided to walk")
                self.state_pub.publish(state_mt3(0, 1, 0, 0))
            # else:
            # if not self.green_dist() == (0,0):
            #     self.state_pub.publish(state_mt3(1, 0, 0, 0))
            # else:
            #     self.state_pub.publish(state_mt3(0, 0, 1, 0))

if __name__ == '__main__':
    try:
        node = Follower()
        node.run()
    except rospy.ROSInterruptException:
        pass   
