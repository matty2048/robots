import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from minitask5.msg import image_proc
import numpy as np
import cv2, cv_bridge
import tf

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class ImageProcessor:
    SLEEP_RATE = 5

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.image_proc_pub = rospy.Publisher('mt5_control', image_proc, queue_size=10)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        self.pos = Position(0, 0, 0)
        self.image_resized = []
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        # self.ranges = [0]*360
        # self.range_min = 0.118
        # self.range_max = 3.5
        self.mask = []
        self.image = []
        self.width = 0
        rospy.init_node('ImageProcessor', anonymous=True)
        cv2.startWindowThread()

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        h = hsv[:, :, 0]
        self.mask = cv2.inRange(h, 50, 70)
        self.image = cv2.bitwise_and(self.image, self.image, mask = self.mask)
        (h, w) = self.image.shape[:2]
        self.width = w
        self.image_resized = cv2.resize(self.image, (int(w/4),int(h/4)))
        cv2.imshow("original", np.array(self.image_resized))
        cv2.waitKey(0)
    
    def callback_laser(self, msg):
        self.ranges = msg.ranges

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

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
    
    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        r.sleep()
        while not rospy.is_shutdown():

            # TODO: PROCESS IMAGE + PUBLISH MSG
            # PUBLISH A LIST OF LOCATIONS WHERE GREEN/RED/BLUE OBJECTS ARE LOCATED
            # 


            r.sleep()

if __name__ == '__main__':
    try:
        image_processor = ImageProcessor()
        image_processor.run()
        pass
    except rospy.ROSInterruptException:
        pass   