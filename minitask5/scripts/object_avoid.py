import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from minitask5.msg import controller

turn = 0
reverse = 0
ranges = []

def callback_laser(msg):
    global reverse, turn
    front = msg.ranges[340:360] + msg.ranges[0:20]
    front_turn = msg.ranges[320:360] + msg.ranges[0:40]
    back = msg.ranges[160:200]

    # if currently turning then keep turning until
    if turn and min(front_turn) < 0.4:
        turn = 1
    # elif reversing keep reversing until
    elif reverse and min(back) < 0.5:
        reverse = 1
    else: 
        if min(back) > 0.5 and min(front) < 0.2:
            reverse = 1
            turn = 0
        elif min(front) < 0.2:
            turn = 1
            reverse = 0
        else:
            turn = 0
            reverse = 0


def callback_control(msg):
    # global turn, turn_direction
    # turn = msg.state_object_avoid
    # turn_direction = 1 if msg.direction_object_avoid else -1
    pass

def avoid():
    global reverse, turn
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback_laser)
    # control_sub = rospy.Subscriber('mt5_control', controller , callback_control)

    rospy.init_node('object_avoid', anonymous=True)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        twist_msg = Twist()
        if reverse:
            twist_msg.linear.x = -0.04
            pub.publish(twist_msg)
        elif turn:
            twist_msg.angular.z = -0.1
            pub.publish(twist_msg)
        else:
            pub.publish(twist_msg)
        r.sleep()

if __name__ == '__main__':
    try:
        avoid()
    except rospy.ROSInterruptException:
        pass
