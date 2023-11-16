import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
import tf
import numpy as np
import math
from matplotlib import pyplot as plt

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class map_navigation():

    def __init__(self):
        self.goalReached = False
        # initiliaze
        self.pos = Position(0, 0, 0)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5
        rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('odom', Odometry, self.callback_odom)
        rospy.init_node('map_navigation', anonymous=False)
        choice = 0
    
    def callback_laser(self, msg):
        self.ranges = msg.ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion) # type: ignore
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Quit program")
        rospy.sleep(1)

    def moveToGoal(self,xGoal,yGoal):

        #define a client for to send goal requests to the move_base server through a SimpleActionClient
        ac = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #wait for the action server to come up
        while(not ac.wait_for_server(rospy.Duration.from_sec(5.0))):
            rospy.loginfo("Waiting for the move_base action server to come up")


        goal = MoveBaseGoal()

        #set up the frame parameters
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # moving towards the goal*/

        goal.target_pose.pose.position =  Point(xGoal,yGoal,0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        rospy.loginfo("Sending goal location ...")
        ac.send_goal(goal)

        ac.wait_for_result(rospy.Duration(60))

        if(ac.get_state() ==  GoalStatus.SUCCEEDED):
            rospy.loginfo("You have reached the destination")
            return True

        else:
            rospy.loginfo("The robot failed to reach the destination")
            return False
    
    def to_grid(self, px, py, origin_, size, resolution):
        offsetx = (1/resolution)*(px - origin_[0])
        offsety = (1/resolution)*(py - origin_[1])
        return (int(offsetx), int(offsety))
        
        
    def to_world(self, gx, gy, origin, size, resolution):
        offsetx = (resolution)*gx + origin[0]
        offsety = (resolution)*gy + origin[1]
        return (offsetx, offsety)

    def to_index(self, gx, gy, size_x):
        return gy * size_x + gx

    def get_line(self, start, end):
        """Bresenham's Line Algorithm
        Produces a list of tuples from start and end
    
        >>> points1 = get_line((0, 0), (3, 4))
        >>> points2 = get_line((3, 4), (0, 0))
        >>> assert(set(points1) == set(points2))
        >>> print points1
        [(0, 0), (1, 1), (1, 2), (2, 3), (3, 4)]
        >>> print points2
        [(3, 4), (2, 3), (1, 2), (1, 1), (0, 0)]
        """
        # Setup initial conditions
        x1, y1 = start
        x2, y2 = end
        dx = x2 - x1
        dy = y2 - y1
    
        # Determine how steep the line is
        is_steep = abs(dy) > abs(dx)
    
        # Rotatself.rangese line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
    
        # Swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
    
        # Recalculate differentials
        dx = x2 - x1
        dy = y2 - y1
    
        # Calcuself.rangeslate error
        error = int(dx / 2.0)
        ystep = 1 if y1 < y2 else -1
    
        # Iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = (y, x) if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += ystep
                error += dx
        # Reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
        return points

    def filter_min_max(self, points):
        points = [x if (x < self.range_max and x > self.range_min) else 0 for x in points]
        return points

    def run(self):
        #self.moveToGoal(-1.45,4.13)
        #self.moveToGoal(-5.5,3.5)
        #self.moveToGoal(4.2,4.2)
        res = 0.1
        size = (25, 25)
        size_x = int(size[0] / res)
        size_y = int(size[1] / res)

        grid = [-1]*(size_x*size_y)
        # origin of grid
        origin_ = (-10, -10)
        # origin of world
        origin = (self.pos.x, self.pos.y)
        # resolution
        r = rospy.Rate(5)
        print("size_x={size_x}, size_y={size_y}, grid size={grid_size}, origin_={pos}".format(size_x=size_x, size_y=size_y, grid_size=np.array(grid).size, pos=origin_))
        print(self.to_grid(self.pos.x, self.pos.y, origin_, size, res))
        
        # while not rospy.is_shutdown():
        for k in range(50):
            points = self.filter_min_max(self.ranges)
            cur_pos = self.to_grid(self.pos.x, self.pos.y, origin_, size, res)
            cur_angle = self.pos.theta
            for i in range(len(points)):
                if points[i] == 0: continue
                rads = radians(i) + cur_angle
                # Suspect this is the cause behind the flipping ?
                endpos = self.to_grid(self.pos.x + points[i] * math.sin(rads), self.pos.y + points[i] * math.cos(rads), origin_, size, res)
                gridpoints = self.get_line(cur_pos, endpos)

                for j in range(len(gridpoints)-1):
                    temp = (gridpoints[j][0], gridpoints[j][1])
                    grid[self.to_index(temp[0], temp[1], size_x)] = 0.05
                temp = (gridpoints[-1][0], gridpoints[-1][1])
                grid[self.to_index(temp[0], temp[1], size_x)] = 1
            r.sleep()
        plt.imshow(np.reshape(np.array(grid), (size_x, size_y)), interpolation='nearest')
        plt.show()

    
if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        thing = map_navigation()
        thing.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")