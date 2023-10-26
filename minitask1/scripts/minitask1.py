
import rospy

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math

class Pose:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class closedloop:
    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.cur_pos.theta = yaw
        self.cur_pos.x = msg.pose.pose.position.x
        self.cur_pos.y = msg.pose.pose.position.y

    def __init__(self):
        rospy.init_node('talker', anonymous=True)
        self.pos_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('odom',Odometry,self.callback_odom)
        self.cur_pos = Pose(0,0,0)
        self.pos_pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
    
    def angle(self, theta):
        return theta if theta > 0 else theta + 2*math.pi # might need to be >= not sure

    def distance(self, pos_1, pos_2):
        return math.dist(pos_1, pos_2)

    def calculate_goal(self, i, pos_1):
        temp = pos_1
        if i == 1:
            temp[0] += 1
        elif i == 2:
            temp[1] += 1
        elif i == 3:
            temp[0] -= 1
        else:
            temp[1] -= 1
        return temp


    def run(self):
        vel_stop = Twist()
        vel_msg1 = Twist()
        vel_msg1.linear.x = 0.05
        vel_turn = Twist()
        vel_turn.angular.z = 0.1
        r = rospy.Rate(10) # 10hz
        n = 4
        self.pos_pub_odom.publish(Odometry())
        for i in range(1, n + 1):
            goal_pos = self.calculate_goal(i, [self.cur_pos.x, self.cur_pos.y])
            while goal_pos - [self.cur_pos.x, self.cur_pos.y] > :
                self.pos_pub.publish(vel_msg1)  
                r.sleep()
            while self.angle(self.cur_pos.theta) < math.fmod((i * math.pi) / 2, 2*math.pi):
                self.pos_pub.publish(vel_turn)
                r.sleep()
        self.pos_pub.publish(vel_stop)
            

def open_loop():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
    t = 0

    num_secs = 5
    vel_msg1 = Twist()
    vel_msg1.linear.x = 0.1
    vel_stop = Twist()

    vel_msg2 = Twist()
    vel_msg2.angular.x = 0
    vel_msg2.angular.y = 0
    vel_msg2.angular.z = 0.31416
    i =0
    
    while not rospy.is_shutdown(): 
        while i < 4:
            t = rospy.Time.now().to_sec()
            while (rospy.Time.now().to_sec() - t) < rospy.Duration(num_secs).to_sec():
                pub.publish(vel_msg1)
                r.sleep()
            t = rospy.Time.now().to_sec()
            while rospy.Time.now().to_sec() - t < rospy.Duration(num_secs).to_sec():
                pub.publish(vel_msg2)
                r.sleep()
            i = i + 1
        pub.publish(vel_stop)
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t  < rospy.Duration(num_secs).to_sec():
            pub.publish(vel_msg1)
            r.sleep()
        t = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - t  < rospy.Duration(num_secs).to_sec():
            pub.publish(vel_msg2)
            r.sleep()

if __name__ == '__main__':
    try:
        loop = closedloop()
        loop.run()
    except rospy.ROSInterruptException:
        pass

