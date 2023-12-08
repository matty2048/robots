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
import tf2_ros as tf2
from struct import * 
import math
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header
from geometry_msgs.msg import Point
import tf2_geometry_msgs as tf2geo
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
        #self.depth_sub = rospy.Subscriber('camera/depth/points', PointCloud2, self.depth_callback)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.obj_pub = rospy.Publisher('/image_proc', image_proc, queue_size=10)
        self.stamp_pub = rospy.Publisher('point2', PointStamped, queue_size=10)
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
        self.buffer = tf2.Buffer()
        self.listener = tf2.TransformListener(self.buffer)
        cv2.startWindowThread()
    

    def callback_laser(self, msg):
        self.ranges = msg.ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max

    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        (h, w) = self.image.shape[:2]
        self.image = cv2.resize(self.image, (int(w/4),int(h/4)))
        # use hsv for green and blue as the boxes are seprable in this color space
        hsv = cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV)
        # use hsl for red as the brick walls confused it with hsv
        hsl = cv2.cvtColor(self.image, cv2.COLOR_BGR2HLS)
        self.width = w
        self.height = h
        self.img_ready = True

        # extract tuned range of HSV and create mask 
        blueLowerValues = np.array([90, 160, 160])
        blueUpperValues = np.array([128, 255, 255])
        self.blueMask = cv2.inRange(hsv, blueLowerValues, blueUpperValues)
        #cv2.imshow("blue mask", self.blueMask)

        greenLowerValues = np.array([40, 200, 20])
        greenUpperValues = np.array([70, 255, 255])
        self.greenMask = cv2.inRange(hsv, greenLowerValues, greenUpperValues)
        
        # red needs 2 masks as it is at both ends of the hue scale
        redLowerValues1 = np.array([0,10, 200])
        redUpperValues1 = np.array([1, 255, 255])
        redLowerValues2 = np.array([170, 10, 200])
        redUpperValues2 = np.array([255, 255, 255])

        redMask1 = cv2.inRange(hsl, redLowerValues1, redUpperValues1)
        redMask2 = cv2.inRange(hsl, redLowerValues2, redUpperValues2)
        self.redMask = redMask1  + redMask2 


        analysis = cv2.connectedComponentsWithStats(self.greenMask, 4, cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        data = []
        for i in range(1, totalLabels):
            pass
            # print("i can see a green goal: ", i)
        

        analysis = cv2.connectedComponentsWithStats(self.redMask, 4, cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        data = []
        for i in range(1, totalLabels):
            # print("i can see a red goal: ", i)
            pass
        
    
    def xytoidx(self, x, y):
        return 1920*y + x
    
    def transform_between_frames(self, p : Point, from_frame, to_frame):
        # transforms from_frame to to_frame using tf2 
        input_pose_stamped = tf2geo.PoseStamped()
        input_pose_stamped.pose.position = p
        input_pose_stamped.header.frame_id = from_frame
        input_pose_stamped.header.stamp = rospy.Time.now()
 
        output_pose_stamped = self.buffer.transform(input_pose_stamped, to_frame, rospy.Duration(1))
        return output_pose_stamped.pose.position


    def depth_callback(self, msg):      
        #need to unpack this into coordinates
        float_iter = iter_unpack("8f", msg.data)
        pointColour = np.zeros((1080,1920))
        pointcloud = np.zeros(((1920 * 1080),4))
        i = 0
        for x in float_iter:
           pointcloud[i][0] = x[0]
           pointcloud[i][1] = x[1]
           pointcloud[i][2] = x[2]
           pointcloud[i][3] = x[4]
           i = i + 1
        self.depth_points = pointcloud

        #get bounding boxes of blue
        analysis = cv2.connectedComponentsWithStats(self.blueMask, 
                                            4, 
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        data = []
        for i in range(1, totalLabels):
            dat = object_data()
            dat.blue = 255
            # get centeroid
            centre = centroid[i]
            centre_x = int(centre[0])
            centre_y = int(centre[1])
            #find centeroid in point cloud
            centre_bot = pointcloud[self.to_idx(centre_x, centre_y, 1920)]
            # make sure depth point is valid
            if math.isnan(centre_bot[0]) or math.isnan(centre_bot[1]): 
                continue
            # convert point from depth camera frame to odom frame
            p = Point(centre_bot[0],centre_bot[1],centre_bot[2])
            p = self.transform_between_frames(p, "camera_depth_optical_frame","odom")
            dat.x_location = p.x
            dat.y_location = p.y
            print(p.x,p.y)
            stamp = PointStamped(header=Header(stamp=rospy.Time.now(),
                                frame_id="odom"),
                                point=Point(dat.x_location, dat.y_location, 0.0))
            self.stamp_pub.publish(stamp)
            #print(dat.x_location, dat.y_location)
            data.append(dat)
        # self.obj_pub.publish(data)

        # get red obj locations
        # get bounding boxes
        analysis = cv2.connectedComponentsWithStats(self.redMask, 
                                            4, 
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        # data = []
        for i in range(1,totalLabels):
            # print("I can see a red goal")
            dat = object_data()
            dat.red = 255
            #get centreoid
            centre = centroid[i]
            centre_x = int(centre[0])
            centre_y = int(centre[1])
            # get centroid depth point
            centre_bot = pointcloud[self.to_idx(centre_x, centre_y, 1920)]
            #make sure depth point is valid
            if math.isnan(centre_bot[0]) or math.isnan(centre_bot[1]): 
                continue
            # convert point from depth camera frame to odom frame
            p = Point(centre_bot[0],centre_bot[1],centre_bot[2])
            p = self.transform_between_frames(p, "camera_depth_optical_frame","odom")
            dat.x_location = p.x
            dat.y_location = p.y
            data.append(dat)

        # #get green obj locations
        # get green object bounding boxes
        analysis = cv2.connectedComponentsWithStats(self.greenMask, 
                                            4, 
                                            cv2.CV_32S)
        (totalLabels, label_ids, stats, centroid) = analysis
        for i in range(1,totalLabels):
            print("I can see a green goal")
            dat = object_data()
            dat.green = 255
            #get centroid 
            centre = centroid[i]
            centre_x = int(centre[0])
            centre_y = int(centre[1])
            # get centroid depth point
            centre_bot = pointcloud[self.to_idx(centre_x, centre_y, 1920)]
            # make sure depth point is valid
            if math.isnan(centre_bot[0]) or math.isnan(centre_bot[1]): 
                continue
            #transform depth point to odom frame
            p = Point(centre_bot[0],centre_bot[1],centre_bot[2])
            p = self.transform_between_frames(p, "camera_depth_optical_frame","odom")
            dat.x_location = p.x
            dat.y_location = p.y
            data.append(dat)
            

        self.depth_ready = True
        
    def to_idx(self, x, y, sizex):
        return y*sizex + x

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
