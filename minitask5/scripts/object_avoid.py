import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from minitask5.msg import controller

turn = 0
turn_direction = 1
ranges = []

def callback_laser(msg):
    global ranges 
    ranges = msg.ranges

def callback_control(msg):
    global turn, turn_direction
    turn = msg.state_object_avoid
    turn_direction = 1 if msg.direction_object_avoid else -1

def avoid():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    scan_sub = rospy.Subscriber('/scan', LaserScan, callback_laser)
    control_sub = rospy.Subscriber('mt5_control', controller , callback_control)

    rospy.init_node('object_avoid', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        global turn, turn_direction
        if turn:
            twist_msg = Twist()
            twist_msg.angular.z = 0.2*turn_direction
            pub.publish(twist_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        avoid()
    except rospy.ROSInterruptException:
        pass
