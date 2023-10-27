import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math
import numpy as np
import queue

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class WallWalk():
    DIR_FORWARD = 0
    DIR_LEFT = 90
    DIR_RIGHT = 270
    DIR_BACK = 180
    WALK_LENGTH = 2.5
    # WALK_ANGLE = 2*math.pi / 3
    WALK_SPEED = 0.2
    WALK_TURN_SPEED = 0.2
    WALK_SECONDS = 10
    SLEEP_RATE = 10


    def __init__(self) -> None:
        rospy.init_node('talker', anonymous=True)
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.pos = Position(0, 0, 0)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y
    
    def callback_laser(self, msg):
        self.ranges = msg.ranges

    def wall_dist(self):
        #a = math.sqrt(pow(self.right_dist(),2) + pow(self.forward_dist(),2))
        #area = (self.right_dist() * self.forward_dist())/2
        #return (2 * area) / a
        points = self.ranges[self.DIR_RIGHT+20:]
        return min(points)


    def lowest_average_reading(self, centre, m):
        points = sorted(self.ranges[(centre - m) % 360:centre - 1:] + self.ranges[centre % 360: (centre + m) % 360:])
        points = [x if x < self.range_max else 3.5 for x in points]
        if len(points):
            return sum(points[:3:]) / 3
    
    def obstacle_condition(self):
        check_angles = [(0, 0.9), (315, 0.45), (335, 0.45), (25, 0.45), (45, 0.45), (90, 0.45)]
        detection_dist = [self.lowest_average_reading(x[0], 10) for x in check_angles if self.lowest_average_reading(x[0], 10) < x[1]]
        ## print(detection_dist)
        return len(detection_dist)

    def obstacle(self,ranges):
        min_dir = ranges.index(min(ranges))
        val = not min_dir > 240  and min(ranges) < 0.5
        return val
    def wall(self,ranges):
        min_dir = ranges.index(min(ranges))
        val = min_dir > 240  and min(ranges) < 0.5
        return val

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        while not rospy.is_shutdown():
            if self.wall(self.ranges):
                start_time = rospy.Time.now().to_sec()
                past_vals = queue.Queue(maxsize=5)
                vel_msg = Twist()
                vel_msg.linear.x = self.WALK_SPEED / 2 
                previous = 0
                while start_time + 10 > rospy.Time.now().to_sec():
                    print("following wall")
                    dist = self.wall_dist()
                    error = dist - 0.5 
                    #print(dist, error)
                    kp = 0.8
                    kd = 0.9
                    ki = 0.5
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
                print("Wall followed!!!")
                #vel_move = Twist()
                #vel_move.linear.x = 1
                #self.pos_pub.publish(vel_move)
                #r.sleep()
        pass

if __name__ == '__main__':
    try:
        ww = WallWalk()
        ww.run()
    except rospy.ROSInterruptException:
        pass

