import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf
import math
import numpy as np

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class ObstacleAvoid():
    WALK_SPEED = 0.2
    WALK_TURN_SPEED = 0.2
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

    def lowest_average_reading(self, centre, m):
        points = sorted(self.ranges[(centre - m) % 360:centre - 1:] + self.ranges[centre % 360: (centre + m) % 360:])
        points = [x if x < self.range_max else 3.5 for x in points]
        if len(points):
            return sum(points[:3:]) / 3
    
    def obstacle_condition(self, conditions):
        detection_dist = [self.lowest_average_reading(x[0], 10) for x in conditions if self.lowest_average_reading(x[0], 10) < x[1]]
        print(detection_dist)
        return len(detection_dist)
    
    
    # def turn(self):
    #     vel_turn = Twist()
    #     r = rospy.Rate(ObstacleAvoid.SLEEP_RATE)
    #     if abs(self.pos.theta - angle) > 0.1:
    #             while abs(self.pos.theta - angle) > 0.03:
    #                 vel_turn.angular.z = ObstacleAvoid.WALK_TURN_SPEED if self.pos.theta < angle else -ObstacleAvoid.WALK_TURN_SPEED
    #                 self.pos_pub.publish(vel_turn)
    #                 r.sleep()   
    #             self.pos_pub.publish(Twist())
    def obstacle(self,ranges):
        min_dir = ranges.index(min(ranges))
        val = not min_dir > 240  and min(ranges) < 0.5
        return val
    def wall(self,ranges):
        min_dir = ranges.index(min(ranges))
        val = min_dir > 240  and min(ranges) < 0.5
        return val

    
    def run(self):
        vel_msg1 = Twist()
        vel_turn = Twist()
        vel_turn.angular.z = ObstacleAvoid.WALK_TURN_SPEED
        vel_msg1.linear.x = ObstacleAvoid.WALK_SPEED
        r = rospy.Rate(ObstacleAvoid.SLEEP_RATE)

        # Turn to correct angle
        cond = [(0, 0.5), (315, 0.45), (335, 0.45), (25, 0.45), (45, 0.45), (90, 0.45)]
        while not rospy.is_shutdown():
            while self.obstacle(self.ranges) and not self.wall(self.ranges):
                    print("avoiding object")
                    # vel_turn.angular.z = ObstacleAvoid.WALK_TURN_SPEED if self.pos.theta < angle else -ObstacleAvoid.WALK_TURN_SPEED
                    self.pos_pub.publish(vel_turn)
                    #print(self.pos.theta)
                    r.sleep()
            #print("AVOIDING DONE!!")
        self.pos_pub.publish(Twist())




if __name__ == '__main__':
    try:
        oa = ObstacleAvoid()
        oa.run()
    except rospy.ROSInterruptException:
        pass

