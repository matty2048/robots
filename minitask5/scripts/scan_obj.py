import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import std_msgs.msg
from minitask5.msg import object_data, image_proc

scan_msg: LaserScan
obj_list: object_data = object_data()

def new_scan_values():
    global scan_msg, obj_list
    for points in scan_msg.ranges:
        pass

def callback_laser(msg):
    global scan_msg
    scan_msg = msg

def callback_object(msg: image_proc):
    global obj_list
    obj_list = msg.object_data

def pub_new_scan():
    global scan_msg
    # pub = rospy.Publisher('/scan_obj', LaserScan, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, callback_laser)
    rospy.Subscriber('/image_proc', image_proc, callback_object)
    rospy.init_node('scan_obj', anonymous=False)
    r = rospy.Rate(2)
    while not rospy.is_shutdown():
        # scan = LaserScan()
        
        # scan.header.stamp = scan_msg.scan_time
        # scan.header.frame_id = "laser_frame"
        # scan.angle_min = scan_msg.angle_min
        # scan.angle_max = scan_msg.angle_max
        # scan.angle_increment = scan_msg.angle_increment
        # scan.time_increment = scan_msg.time_increment
        # scan.range_min = scan_msg.range_min
        # scan.range_max = scan_msg.range_max

        # scan.ranges = [1]*360
        # scan.intensities = scan_msg.intensities
        # pub.publish(scan_msg)

        r.sleep()

if __name__ == '__main__':
    try:
        pub_new_scan()
    except rospy.ROSInterruptException:
        pass
