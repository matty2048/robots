from math import isnan, nan
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan, Image
from minitask5.msg import object_data, image_proc
import numpy as np
import pylab as plt
from PIL import Image as im
import random
import cv2, cv_bridge
from minitask5.msg import image_proc
from minitask5.msg import object_data
import queue
import tf
from struct import * 
import math
class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta


class Point_rgb:
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
        self.obj_pub = rospy.Publisher('/image_proc', image_proc, queue_size=10)
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
        self.pos = Position(0,0,0)
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
        cv2.imshow("blue mask", self.blueMask)
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

        #self.image = cv2.bitwise_and(self.image, self.image, mask = self.greenMask)
        
        self.width = w
        self.height = h
        #cv2.imshow("image", self.image)
        self.img_ready = True
    
    def xytoidx(self, x, y):
        return 1920*y + x
    
    def depth_callback(self, msg):      
        #4147200
        #2073600
        #cache the current masks or data will be wacky
        bluemask = self.blueMask
        redmask = self.redMask
        greenmask = self.greenMask
        # print(msg.point_step)
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
        #get bounding boxes of blue
        analysis = cv2.connectedComponentsWithStats(bluemask, 
                                            4, 
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        data = []
        for i in range(1, totalLabels):
            dat = object_data()
            dat.blue = 255
            centre = centroid[i]
            centre_x = int(centre[0] * 4)
            centre_y = int(centre[1] * 4)
            centre_bot = pointcloud[self.to_idx(centre_x, centre_y, 1920)][0:2]
            if math.isnan(centre_bot[0]) or math.isnan(centre_bot[1]): 
                continue
            centre_rotated = (centre_bot[0] * math.cos(-self.pos.theta) - centre_bot[1] * math.sin(-self.pos.theta), 
                              centre_bot[0] * math.sin(-self.pos.theta) + centre_bot[1] * math.cos(-self.pos.theta))
            centre_world = (centre_rotated[0] + self.pos.x, centre_rotated[1] + self.pos.y)
            dat.x_location = centre_world[0]
            dat.y_location = centre_world[1]
            print(dat.x_location, dat.y_location)
            data.append(dat)
        # self.obj_pub.publish(data)

        # get red obj locations
        analysis = cv2.connectedComponentsWithStats(redmask, 
                                            4, 
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        # data = []
        for i in range(1,totalLabels):
            dat = object_data()
            dat.red = 255
            centre = centroid[i]
            centre_x = int(centre[0] * 4)
            centre_y = int(centre[1] * 4)
            centre_bot = pointcloud[self.to_idx(centre_x, centre_y, 1920)][0:2]
            if math.isnan(centre_bot[0]) or math.isnan(centre_bot[1]): 
                continue
            centre_rotated = (centre_bot[0] * math.cos(-self.pos.theta) - centre_bot[1] * math.sin(-self.pos.theta), 
                              centre_bot[0] * math.sin(-self.pos.theta) + centre_bot[1] * math.cos(-self.pos.theta))
            centre_world = (centre_rotated[0] + self.pos.x, centre_rotated[1] + self.pos.y)
            dat.x_location = centre_world[0]
            dat.y_location = centre_world[1]
            data.append(dat)
        # self.obj_pub.publish(data)

        #get green obj locations
        analysis = cv2.connectedComponentsWithStats(greenmask, 
                                            4, 
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        # data = []
        for i in range(1,totalLabels):
            dat = object_data()
            dat.green = 255
            centre = centroid[i]
            centre_x = int(centre[0] * 4)
            centre_y = int(centre[1] * 4)
            centre_bot = pointcloud[self.to_idx(centre_x, centre_y, 1920)][0:2]
            if math.isnan(centre_bot[0]) or math.isnan(centre_bot[1]): 
                continue
            centre_rotated = (centre_bot[0] * math.cos(-self.pos.theta) - centre_bot[1] * math.sin(-self.pos.theta), 
                              centre_bot[0] * math.sin(-self.pos.theta) + centre_bot[1] * math.cos(-self.pos.theta))
            centre_world = (centre_rotated[0] + self.pos.x, centre_rotated[1] + self.pos.y)
            dat.x_location = centre_world[0]
            dat.y_location = centre_world[1]
            data.append(dat)
        self.obj_pub.publish(data)

        #mask_indices = np.transpose(np.where(self.blueMask != 0))
        #print(mask_indices)
        #for i,j in mask_indices:
        #    depth_point = pointcloud[1920*j + i]
        #    theta = -self.pos.theta
            

        self.depth_ready = True
        
    def to_idx(self, x, y, sizex):
        return y*sizex + x
    def getBlueIdxs(self):
        return np.transpose(np.nonzero(self.blueMask))

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y


    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
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
