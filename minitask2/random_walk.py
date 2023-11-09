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
    WALK_LENGTH = 2.5
    # WALK_ANGLE = 2*math.pi / 3
    WALK_SPEED = 0.2
    WALK_TURN_SPEED = 0.2
    WALK_SECONDS = 10
    DIR_FORWARD = 0
    DIR_LEFT = 90
    DIR_RIGHT = 270
    DIR_BACK = 180
    SLEEP_RATE = 10


    def __init__(self) -> None:
        rospy.init_node('talker', anonymous=True)
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        self.pos = Position(0, 0, 0)
        self.ranges = [1]*360
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

    def lowest_average_reading(self, centre, m):
        points = sorted(self.ranges[(centre - m) % 360:centre - 1:] + self.ranges[centre % 360: (centre + m) % 360:])
        points = [x if x < self.range_max else 3.5 for x in points]
        if len(points):
            return sum(points[:3:]) / 3
        
    def obstacle_condition(self, conditions):
        check_angles = [(0, 0.4), (315, 0.3), (335, 0.4), (25, 0.4), (45, 0.3), (90, 0.4)]
        detection_dist = [self.lowest_average_reading(x[0], 10) for x in conditions if self.lowest_average_reading(x[0], 10) < x[1]]
        ## print(detection_dist)
        return len(detection_dist)

    def dist(self, pos_1, pos_2):
        return math.dist(pos_1, pos_2)

    def obstacle(self,ranges):
        min_dir = ranges.index(min(ranges))
        val = not min_dir > 240  and min(ranges) < 0.5
        return val
    def wall(self,ranges):
        min_dir = ranges.index(min(ranges))
        val = min_dir > 240  and min(ranges) < 0.5
        return val


    def turnToAngle(self, angle):
        vel_turn = Twist()
        r = rospy.Rate(RandomWalk.SLEEP_RATE)
        if abs(self.pos.theta - angle) > 0.1:
                while abs(self.pos.theta - angle) > 0.03 and (not self.obstacle(self.ranges) and not self.wall(self.ranges)):
                    vel_turn.angular.z = RandomWalk.WALK_TURN_SPEED if self.pos.theta < angle else -RandomWalk.WALK_TURN_SPEED
                    self.pos_pub.publish(vel_turn)
                    print("random walking")
                    r.sleep()   
                self.pos_pub.publish(Twist())
    
    def walkForward(self, cond):
        vel_msg1 = Twist()
        vel_msg1.linear.x = RandomWalk.WALK_SPEED
        r = rospy.Rate(RandomWalk.SLEEP_RATE)
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t < rospy.Duration(RandomWalk.WALK_SECONDS).to_sec() and (not self.obstacle(self.ranges) and not self.wall(self.ranges)):
            self.pos_pub.publish(vel_msg1)
            if not self.obstacle() and not self.wall():
                self.pos_pub.publish(Twist())
                break
            r.sleep()
    
    def run(self):
        vel_stop = Twist()
        vel_msg1 = Twist()
        vel_msg1.linear.x = RandomWalk.WALK_SPEED
        r = rospy.Rate(10) # 10hz
        r.sleep()
        while not rospy.is_shutdown():
            # Turns to angle given
            angle = random.uniform(-math.pi, math.pi)
            self.turnToAngle(angle)

            # Walks forward until obstacle or time complete
            cond = [(0, 0.4), (315, 0.3), (335, 0.4), (25, 0.4), (45, 0.3), (90, 0.4)]
            self.walkForward(cond)
            #self.pos_pub.publish(vel_stop)

            # sleeps until obstacle cleared
            #print("Obstacle detected")
            
            while self.obstacle(self.ranges) or self.wall(self.ranges):
                r.sleep()
            angle = random.uniform(-math.pi, math.pi)


if __name__ == '__main__':
    try:
        rw = RandomWalk()
        rw.run()
    except rospy.ROSInterruptException:
        pass

