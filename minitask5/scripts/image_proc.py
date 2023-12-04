from math import isnan, nan
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan, Image
import numpy as np
import pylab as plt
from PIL import Image as im
import random
import cv2, cv_bridge
import queue
import tf
from struct import * 
class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta


class Point:
    def __init__(self, x, y, z, rgb) -> None:
        self.x : float = x
        self.y : float = y
        self.z : float = z
        self.rgb : float = rgb

class Camera:
    SLEEP_RATE = 5
    def __init__(self) -> None:
        self.bridge = cv_bridge.CvBridge()
        #cv2.namedWindow("original", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('camera/depth/points', PointCloud2, self.depth_callback)

        self.image_resized = []
        self.blueMask = []
        self.redMask = []
        self.greenMask = []
        self.image = []
        self.width = 0
        self.height = 0
        self.depth_points = []
        self.depth_ready = False
        self.img_ready = False
        cv2.startWindowThread()
    
    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = self.image.shape[:2]
        self.image = cv2.resize(self.image, (int(w/4),int(h/4)))
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        hsl = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)
        #h = hsv[:, :, 0]
        blueLowerValues = np.array([90, 160, 160])
        blueUpperValues = np.array([128, 255, 255])
        self.blueMask = cv2.inRange(hsv, blueLowerValues, blueUpperValues)
        
        greenLowerValues = np.array([40, 200, 20])
        greenUpperValues = np.array([70, 255, 255])
        self.greenMask = cv2.inRange(hsv, greenLowerValues, greenUpperValues)
        
        redLowerValues1 = np.array([0,10, 200])
        redUpperValues1 = np.array([1, 255, 255])
        redLowerValues2 = np.array([170, 10, 200])
        redUpperValues2 = np.array([255, 255, 255])

        redMask1 = cv2.inRange(hsl, redLowerValues1, redUpperValues1)
        redMask2 = cv2.inRange(hsl, redLowerValues2, redUpperValues2)
        self.redMask = redMask1  + redMask2 

        self.image = cv2.bitwise_and(self.image, self.image, mask = self.greenMask)
        # open the image with a kernel
        
        self.width = w
        self.height = h
        cv2.imshow("image", self.image)
        self.img_ready = True
    
    def xytoidx(self, x, y):
        return 1920*y + x
    
    def depth_callback(self, msg):      
        for f in msg.fields:
            print(f.name, f.offset, f.datatype, f.count)
        #4147200
        #2073600
        print(msg.point_step)
        float_iter = iter_unpack("8f", msg.data)
        #l = len(list(float_iter))
        #print(l)
        pointColour = np.zeros((1080,1920))
        pointcloud = np.zeros(((1920 * 1080),4))
        i = 0
        for x in float_iter:
           pointcloud[i][0] = x[0]
           pointcloud[i][1] = x[1]
           pointcloud[i][2] = x[2]
           pointcloud[i][3] = x[4]
           #print(x[4])
           i = i + 1
        self.depth_points = pointcloud
        self.depth_ready = True
        

    def getBlueIdxs(self):
        return np.transpose(np.nonzero(self.blueMask))
         

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        r.sleep()
        while not rospy.is_shutdown():

            if not (self.depth_ready and self.img_ready):
                r.sleep()
                continue
            #indexes = self.getBlueIdxs()
            #if indexes.any():
            #    print(self.depth_points[self.xytoidx(indexes[0][1] * 4,indexes[0][0] * 4)][2])
            r.sleep()
        pass



if __name__ == '__main__':
    try:
        rospy.init_node('follower')
        camera = Camera()
        camera.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
