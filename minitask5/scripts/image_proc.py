from math import isnan, nan
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import LaserScan, Image
from minitask5.msg import object_data, image_proc
import numpy as np
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
        self.obj_loc_pub = rospy.Publisher('/image_proc', image_proc, queue_size=10)
        self.image_resized = []
        self.mask = []
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
        #h = hsv[:, :, 0]
        lowerValues = np.array([90, 160, 160])
        upperValues = np.array([128, 255, 255])
        self.mask = cv2.inRange(hsv, lowerValues, upperValues)
        self.image = cv2.bitwise_and(self.image, self.image, mask = self.mask)
        # open the image with a kernel
        
        self.width = w
        self.height = h
        cv2.imshow("opened image", self.image)
        self.img_ready = True
    
    def xytoidx(self, x, y):
        return 1920*y + x
    
    def depth_callback(self, msg):      
        # for f in msg.fields:
        #     print(f.name, f.offset, f.datatype, f.count)
        #4147200
        #2073600
        # print(msg.point_step)
        float_iter = iter_unpack("8f", msg.data)
        #l = len(list(float_iter))
        #print(l)
        
        pointcloud = np.zeros(((1920 * 1080),4))
        i = 0
        for x in float_iter:
           pointcloud[i][0] = x[0]
           pointcloud[i][1] = x[1]
           pointcloud[i][2] = x[2]
           pointcloud[i][3] = x[4]
           i = i + 1
        self.depth_points = pointcloud
        self.depth_ready = True

    def getBlueIdxs(self):
        return np.transpose(np.nonzero(self.mask))
         

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)

        TEMP_OBJECT_LIST = image_proc(object_data = [
            object_data(blue= 255, x_location= -0.329, y_location= 0.0587, rough_size= 0.70),
            object_data(blue= 255, x_location= 0.269, y_location= -0.6706, rough_size= 0.70),
            object_data(blue= 255, x_location= 0.316, y_location= 0.6272, rough_size= 0.70),
            object_data(blue= 255, x_location= 0.9469, y_location= -3.15, rough_size= 0.70),
        ])
        while not rospy.is_shutdown():

            if not (self.depth_ready and self.img_ready):
                r.sleep()
                continue
            #indexes = self.getBlueIdxs()
            #if indexes.any():
            #    print(self.depth_points[self.xytoidx(indexes[0][1] * 4,indexes[0][0] * 4)][2])
            self.obj_loc_pub.publish(TEMP_OBJECT_LIST)
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
