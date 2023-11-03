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
    DIR_FORWARD = 0
    DIR_LEFT = 90
    DIR_RIGHT = 270


    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        cv2.namedWindow("original", 1)
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
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
        count = 0
        xsum = 0
        ysum = 0
        h, w, _ = self.image_resized.shape
        # calculate moments of binary image
        M = cv2.moments(self.mask)

        # calculate x,y coordinate of center
        cX = int(M["m10"] / (M["m00"] + 1))
        cY = int(M["m01"] / (M["m00"] + 1))

        centre_mass = (cX, cY)
        cv2.circle(self.image, centre_mass, 10, (0,0,255), 2)
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
    
    def obstacle_dist(self):
        #a = math.sqrt(pow(self.right_dist(),2) + pow(self.forward_dist(),2))
        #area = (self.right_dist() * self.forward_dist())/2
        #return (2 * area) / a
        points = self.ranges[len(self.ranges)-45:] + self.ranges[:self.DIR_FORWARD + 45]
        return min(points)

    def run(self):
        vel_msg1 = Twist()
        vel_turn = Twist()
        vel_green = Twist()
        vel_green.linear.x = self.WALK_SPEED
        past_vals = queue.Queue(maxsize=5)
        vel_turn.angular.z = self.WALK_TURN_SPEED
        vel_msg1.linear.x = self.WALK_SPEED

        vel_green2 = Twist()
        vel_green2.linear.x = self.WALK_SPEED / 3
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
            cond = [(0, 1), (315, 0.40), (335, 0.75), (25, 0.75), (45, 0.40), (90, 0.45)]
            
            if (not self.green_dist() == (0,0)):
                print("Going towards green")
                previous = 0
                while not self.obstacle_condition(cond) and (not self.green_dist() == (0,0)):
                    dist = self.green_dist()
                    if not dist or dist == (0,0): continue
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
                    vel_green.angular.z = kp*P + kd*D + I
                    self.pos_pub.publish(vel_green)
                    r.sleep()

                while self.obstacle_condition(cond) and (not self.green_dist() == (0,0)):
                    # pid to avoid obstacle 
                    dist = self.obstacle_dist()
                    #if not dist or dist == (0,0): continue
                    error = dist - 0.1
                    print(dist, error)
                    kp = 0.2
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
                    vel_green2.angular.z = kp*P + kd*D + I
                    self.pos_pub.publish(vel_green2)
                    r.sleep()
            else:
                print("Random walking")
                if self.obstacle_condition(cond):
                    while self.obstacle_condition(cond):
                        self.pos_pub.publish(vel_turn)
                        r.sleep()
                
                while not self.obstacle_condition(cond) and (self.green_dist() == (0,0)):
                    self.pos_pub.publish(vel_msg1)
                    if self.obstacle_condition(cond):
                        self.pos_pub.publish(Twist())
                        break
                    r.sleep()
                # random turn for set time or if there is an obstacle
                t = rospy.Time.now().to_sec()
                r_t = random.randint(3, 10)
                while (rospy.Time.now().to_sec() - t < rospy.Duration(r_t).to_sec() or self.obstacle_condition(cond)) and self.green_dist() == (0,0):
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