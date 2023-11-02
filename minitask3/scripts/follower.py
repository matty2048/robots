import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
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
    SLEEP_RATE = 2
    WALK_SPEED = 0.3
    WALK_TURN_SPEED = 0.3
    WALK_SECONDS = 10

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.pos = Position(0, 0, 0)
        self.image_resized = None
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5
        
    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = image.shape[:2]
        self.image_resized = cv2.resize(image, (w//4,h//4))
        cv2.imshow("original", self.image_resized)
        cv2.waitKey(0)

    def callback_laser(self, msg):
        self.ranges = msg.ranges

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    def green_location(self):
        th1 = cv2.inRange(self.image_resized, (36, 25, 25), (70, 255,255))
        mass_x, mass_y = np.where(th1 >= 200)
        # mass_x and mass_y are the list of x indices and y indices of mass pixels
        cent_x = np.average(mass_x)
        cent_y = np.average(mass_y)
        return (cent_x, cent_y)
    
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
        vel_msg1 = Twist()
        vel_turn = Twist()
        vel_turn.angular.z = self.WALK_TURN_SPEED
        vel_msg1.linear.x = self.WALK_SPEED
        r = rospy.Rate(self.SLEEP_RATE)
        r.sleep()
        while not rospy.is_shutdown():
            # Re arrange later to
            # if green detected
            #     move towards green
            #     navigate around obstacles
            # else
            #     random walk until green detected

            # Check for obstacles -- Obstacle Avoid
            cond = [(0, 0.5), (315, 0.45), (335, 0.45), (25, 0.45), (45, 0.45), (90, 0.45)]
            if self.obstacle_condition(cond):
                while self.obstacle_condition(cond):
                    self.pos_pub.publish(vel_turn)
                    r.sleep()
            
            # Walk towards green
            if not self.obstacle_condition(cond): # and can see green:
                # walk towards green

                # reach green, end
                pass
            
            # walk forward
            while not self.obstacle_condition(cond):
                self.pos_pub.publish(vel_msg1)
                if self.obstacle_condition(cond):
                    self.pos_pub.publish(Twist())
                    break
                r.sleep()

            # random turn for set time or if there is an obstacle
            t = rospy.Time.now().to_sec()
            r_t = random.randint(3, 10)
            while rospy.Time.now().to_sec() - t < rospy.Duration(r_t).to_sec() or self.obstacle_condition(cond):
                self.pos_pub.publish(vel_turn)

                r.sleep()
            
            self.pos_pub.publish(Twist())
            r.sleep()
            



if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        follower = Follower()
        follower.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
