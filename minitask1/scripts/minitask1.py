
import rospy

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np

class Pose:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class closedloop:
    LENGTH = 1.05
    TURNS = 4

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

    def angle(self, theta):
        return theta if theta > 0 else theta + 2*math.pi # might need to be >= not sure

    def generate_goals(self, pos_1):
        S = [closedloop.LENGTH, 0]
        goals = [[0,0]]
        theta = 0
        for i in range(0, 3):
            A = [[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]]
            pos_1 += np.matmul(A, S)
            goals.append(np.round(pos_1, 4))
            theta += math.pi / 2
        return goals

    def generate_directions(self, goals):
         directions = [goals[(i + 1) % len(goals)] - goals[i] for i in range(len(goals))]
         # return [v / np.linalg.norm(v) for v in directions]
         return directions

    def run(self):
        angles = [math.pi/2, math.pi, -math.pi/2, 0]
        vel_stop = Twist()
        vel_msg1 = Twist()
        vel_msg1.linear.x = 0.1
        vel_turn = Twist()
        vel_turn.angular.z = 0.2
        r = rospy.Rate(10) # 20hz
        S = [closedloop.LENGTH, 0]
        directions = self.generate_directions(self.generate_goals([self.cur_pos.x, self.cur_pos.y]))
        for i in range(0, closedloop.TURNS):

            goal_pos = [self.cur_pos.x, self.cur_pos.y] + directions[i]
            print(directions[i])
            #print(self.angle(self.cur_pos.theta), (i+1) *math.pi / 2)
            while math.dist(goal_pos, [self.cur_pos.x, self.cur_pos.y]) > 0.1:
                print(goal_pos, [self.cur_pos.x, self.cur_pos.y])
                print(math.dist(goal_pos, [self.cur_pos.x, self.cur_pos.y]))
                print()
                self.pos_pub.publish(vel_msg1)  
                r.sleep()
            self.pos_pub.publish(vel_stop)
            lastdelta = self.angle(self.cur_pos.theta) - self.angle(angles[i])
            while np.sign(self.angle(self.cur_pos.theta) - self.angle(angles[i])) == np.sign(lastdelta):
                lastdelta = self.angle(self.cur_pos.theta) - self.angle(angles[i])
            
                print(self.cur_pos.theta, (i+1) *math.pi / 2)
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
    vel_msg1.linear.x = 0.2
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

