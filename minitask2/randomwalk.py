import rospy
import tf
import random
import math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
 
class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta
 
class RandomWalk():
    WALK_LENGTH = 0.7
    # WALK_ANGLE = 2*math.pi / 3
    WALK_SPEED = 0.1
    WALK_TURN_SPEED = 0.1
    DIR_FORWARD = 0
    DIR_LEFT = 90
    DIR_RIGHT = 270
    DIR_BACK = 180
 
 
    def __init__(self) -> None:
        rospy.init_node('talker', anonymous=True)
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.pos = Position(0, 0, 0)
        self.ranges = [0]*360
        self.range_min = 0
        self.range_max = 0

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y
 
    def callback_laser(self, msg):
        self.ranges = msg.ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max

 
    def lowest_average_reading(self, centre, m):
        points = sorted(self.ranges[(centre - m) % 360:centre - 1:] + self.ranges[centre % 360: (centre + m) % 360:])
        points = [x if x < self.range_max and x > self.range_min else 3.5 for x in points]
        if len(points):
            return sum(points[:5:]) / len(points[:5:])
 
    def obstacle_detected(self):   
        return self.lowest_average_reading(RandomWalk.DIR_FORWARD, 10) < 0.4 or \
                self.lowest_average_reading((RandomWalk.DIR_FORWARD - 45 % 360), 10) < 0.35 or \
                self.lowest_average_reading((RandomWalk.DIR_FORWARD - 25 % 360), 10) < 0.35 or \
                self.lowest_average_reading((RandomWalk.DIR_FORWARD + 45 % 360), 10) < 0.35 or \
                self.lowest_average_reading((RandomWalk.DIR_FORWARD + 25 % 360), 10) < 0.35

 
    def dist(self, pos_1, pos_2):
        return math.dist(pos_1, pos_2)
 
    def run(self):
        vel_stop = Twist()
        vel_msg1 = Twist()
        vel_msg1.linear.x = RandomWalk.WALK_SPEED
        vel_turn = Twist()
        r = rospy.Rate(10) # 10hz
        r.sleep()
        start_pos = [self.pos.x, self.pos.y]
        angle = random.uniform(-math.pi, math.pi)
        print(angle)
        while not rospy.is_shutdown():
            if abs(self.pos.theta - angle) > 0.1:
                while abs(self.pos.theta - angle) > 0.03:
                    print(self.pos.theta - angle)
                    print(angle)
                    print()
                    vel_turn.angular.z = RandomWalk.WALK_TURN_SPEED if self.pos.theta < angle else -RandomWalk.WALK_TURN_SPEED
                    self.pos_pub.publish(vel_turn)
                    r.sleep()   
                self.pos_pub.publish(vel_stop)

            print(start_pos, [self.pos.x, self.pos.y])
            if self.dist(start_pos, [self.pos.x, self.pos.y]) < RandomWalk.WALK_LENGTH:
                self.pos_pub.publish(vel_msg1)
                r.sleep() 
            else:
                start_pos = [self.pos.x, self.pos.y]
                angle = random.uniform(-math.pi, math.pi)
                print(angle)
 
            if self.obstacle_detected():
                ##### Break --- Obstacle
                ### temp testing code below
                self.pos_pub.publish(vel_stop)
                angle = random.uniform(-math.pi, math.pi)
                print(angle)
                # break
            r.sleep()
 
if __name__ == '__main__':
    try:
        rw = RandomWalk()
        rw.run()
    except rospy.ROSInterruptException:
        pass