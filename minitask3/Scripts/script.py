import rospy
from sensor_msgs.msg import Image
import cv2, cv_bridge
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math
import numpy as np
import queue

class Follower:
    def __init__(self):
      self.bridge = cv_bridge.CvBridge()
      cv2.namedWindow("original", 1)
      self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
      self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
      self.image_resized = []
      self.mask = []
      self.image = []
      self.width = 0
      cv2.startWindowThread()

    def green_dist(self):
      if(np.size(self.image_resized) == 0): return
      count = 0
      xsum = 0
      ysum = 0
      h, w, _ = self.image_resized.shape

    #   for i in range(h):
    #      for j in range(w):
    #         b,g,r = self.image_resized[i,j]
    #         if(g > 0):
    #            count += 1
    #            xsum += j
    #            ysum += i
    # calculate moments of binary image
      M = cv2.moments(self.mask)
 
    # calculate x,y coordinate of center
      cX = int(M["m10"] / (M["m00"] + 1))
      cY = int(M["m01"] / (M["m00"] + 1))

      centre_mass = (cX, cY)
      cv2.circle(self.image, centre_mass, 10, (0,0,255), 2)
      #print(centre_mass)
      return centre_mass
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
      #cv2.waitKey(0)

    def run(self):
        r = rospy.Rate(10)
        past_vals = queue.Queue(maxsize=5)
        vel_msg = Twist()
        vel_msg.linear.x = 0.2 
        previous = 0
        while True:
            
            #print("following wall")
            dist = self.green_dist()
            if not dist or dist == (0,0): continue
            #if len(self.image) != 0:
            #    (h, w) = self.image.shape[:2]
            #    image_resized = cv2.resize(self.image, (int(w/4),int(h/4)))
            #    cv2.imshow("circled", image_resized)
            #    #cv2.waitKey()

            error = dist[0] - int(self.width/2)
            print(dist, error)
            kp = 0.0002
            kd = 0.000
            ki = 0.000
            P = -error
            I = ki * (P - previous)
            previous = P
            sums =0   
            for i in past_vals.queue:
                sums += i
            D = sums / len(past_vals.queue) if sums != 0 else 0
            if past_vals.full(): 
                past_vals.get()
            past_vals.put(P)
            vel_msg.angular.z = kp*P + kd*D + I
            #print(vel_msg.angular.z) 
            #print(P)                   
            #print()
            self.pos_pub.publish(vel_msg)
            r.sleep()
        
rospy.init_node('follower')
follower = Follower()
follower.run()