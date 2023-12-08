import rospy
from math import radians, degrees
from actionlib_msgs.msg import *
from geometry_msgs.msg import Quaternion, PoseWithCovarianceStamped, PoseWithCovariance, Pose
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap, SetMap
from sensor_msgs.msg import LaserScan
from minitask5.msg import object_data, image_proc
import tf
import numpy as np
import math

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class map_navigation():
    ROBOT_WIDTH = 0.2
    BLUE_TILE_X_CORR = 0
    BLUE_TILE_Y_CORR = -0.3
    BLUE_TILE_SIZE = 0
    BLUE_TILE_LIST = [
        object_data(blue= 255, x_location= -0.329, y_location= 0.0587+ BLUE_TILE_Y_CORR, rough_size= 0.50 + BLUE_TILE_SIZE),
        object_data(blue= 255, x_location= 0.269, y_location= -0.6706+ BLUE_TILE_Y_CORR, rough_size= 0.50 + BLUE_TILE_SIZE),
        object_data(blue= 255, x_location= 0.316, y_location= 0.6272+ BLUE_TILE_Y_CORR, rough_size= 0.50 + BLUE_TILE_SIZE),
        object_data(blue= 255, x_location= 0.9469, y_location= -3.15+ BLUE_TILE_Y_CORR, rough_size= 0.50 + BLUE_TILE_SIZE),
    ]

    def __init__(self, res = 0.05, origin = (-10, -10), threshold = 0.65):
        self.old_obj_loc: object_data = []
        self.new_obj_loc: object_data = []
        self.obj_loc_changed = True
        self.pos = Position(0,0,0)
        self.ranges = [0]*360
        self.range_min = 0.118
        self.range_max = 3.5
        self.threshold = threshold
        self.old_grid = OccupancyGrid()
        self.objects_found = []
        self.past_grids = []
        rospy.Subscriber('/scan', LaserScan, self.callback_laser)
        rospy.Subscriber('/odom', Odometry, self.callback_odom)
        rospy.Subscriber('/image_proc', image_proc, self.callback_object)
        rospy.init_node('map_navigation', anonymous=True)

        node_name= rospy.get_name()
        map_provided = rospy.get_param(node_name +'/map_provided')
        pub_to = rospy.get_param(node_name + '/pub_to')
        self.occ_pub = rospy.Publisher(pub_to, OccupancyGrid, queue_size=10)
        self.corr_x = rospy.get_param(node_name + '/corr_x')
        self.size_x = rospy.get_param(node_name + '/size_x')
        self.size_y = rospy.get_param(node_name + '/size_y')
        self.buffer = rospy.get_param(node_name + '/buffer_size')
        self.res = rospy.get_param(node_name + '/res')
        self.blue_tiles_provided = rospy.get_param(node_name + '/blue_tiles_provided')
        self.grid : list[int] = [-1]*self.size_x*self.size_y
        self.origin_ = origin
        if map_provided:
            try:
                current_map = self.get_map_client()
                self.origin_ = (current_map.info.origin.position.x, current_map.info.origin.position.y)
                self.grid = self.adjust_map(int(0/self.res), int(0/self.res), self.size_x, list(current_map.data))
            except:
                print("Failed to retrieve map")
    
    def callback_object(self, msg: image_proc):
        # if self.new_obj_loc== msg.object_data: return
        self.old_obj_loc = self.new_obj_loc
        self.new_obj_loc = msg.object_data
        # self.obj_loc_changed = True

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
    
    def get_map_client(self):
        rospy.wait_for_service('/static_map')
        try:
            static_map = rospy.ServiceProxy('/static_map', GetMap)
            resp1 = static_map()
            # print(list(resp1.map.data))
            return resp1.map
        except rospy.ServiceException as e:
            # print("Service call failed: %s"%e)
            pass
    
    def adjust_map(self, x_cells, y_cells, size_x, old_map):
        new_map = np.reshape(np.array(old_map), (-1, size_x))
        new_map = np.roll(new_map, x_cells, axis=0)
        new_map = np.roll(new_map, y_cells, axis=1)
        return new_map.flatten().tolist()
    
    def to_grid(self, px, py, origin_, resolution):
        offsetx = (1/resolution)*(px - origin_[0] + self.corr_x)
        offsety = (1/resolution)*(py - origin_[1])
        return (int(offsetx), int(offsety))
        
        
    def to_world(self, gx, gy, origin, resolution):
        offsetx = (resolution)*gx + origin[0] - self.corr_x
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

    def renew_objects(self):
        # if not self.obj_loc_changed: return
        if self.blue_tiles_provided:
            self.change_all_objects_to(self.BLUE_TILE_LIST, 0)
            self.change_all_objects_to(self.BLUE_TILE_LIST, 100)
        # self.obj_loc_changed = False

    def change_all_objects_to(self, obj_list, probability):
        # CHANGE IN FULL IMPLEMENTATION
        for i in range(len(self.past_grids)):
            for tile in obj_list:
                self.past_grids[i] = self.map_object_value_to(tile, self.past_grids[i],probability)
                # self.grid = self.map_object_value_to(tile, self.grid ,probability)

    def map_object_value_to(self, object_found: object_data, grid, probability=100):
        size_r = int(object_found.rough_size  / 2*(1/self.res))
        centre = self.to_grid(object_found.x_location, object_found.y_location + self.corr_x, self.origin_, self.res)
        for i in range(-size_r, size_r):
            for j in range(-size_r, size_r):
                grid[self.to_index(centre[0] + i, centre[1] + j, self.size_x)] = probability
        return grid

    def pub_occ_grid(self, past_grids):
        self.old_grid.data = self.grid
        self.old_grid.info.height = self.size_y
        self.old_grid.info.width = self.size_x
        self.old_grid.info.resolution = self.res
        self.old_grid.info.origin.position.x = self.origin_[0]
        self.old_grid.info.origin.position.y = self.origin_[1]
        publish_grid = [int(sum(x)/len(past_grids)) for x in zip(*past_grids)]
        publish_grid = [x if x >= 0 else -1 for x in publish_grid]

        # publish_grid = list[int] = [-1]*self.size_x*self.size_y
        self.old_grid.data = publish_grid
        self.occ_pub.publish(self.old_grid)
        return

    def map_all_lines(self):
        points = self.filter_min_max(self.ranges)
        cur_pos = (self.pos.x, self.pos.y)
        cur_pos_grid = self.to_grid(self.pos.x, self.pos.y, self.origin_, self.res)
        cur_angle = self.pos.theta
        for i in range(len(points)):
            if points[i] == 0: continue
            if (abs(points[i] - points[(i + 1) % 360]) > 0.15 ) and (abs(points[i] - points[(i - 1) %360]) > 0.15 ): continue
            rads = (radians(i) + cur_angle - math.pi/2) 
            endpos = self.to_grid(cur_pos[0] + (points[i]) * -math.sin(rads), cur_pos[1] + (points[i]) * math.cos(rads), self.origin_, self.res)
            gridpoints = self.get_line(cur_pos_grid, endpos)
            for j in range(len(gridpoints)-2):
                temp = (gridpoints[j][0], gridpoints[j][1])
                self.grid[self.to_index(temp[0], temp[1], self.size_x)] = 0

        for i in range(len(points)):
            if points[i] == 0: continue
            rads = (radians(i) + cur_angle - math.pi/2) 
            endpos = self.to_grid(cur_pos[0] + (points[i]) * -math.sin(rads), cur_pos[1] + (points[i]) * math.cos(rads), self.origin_, self.res)
            gridpoints = self.get_line(cur_pos_grid, endpos)
            for i in range(self.buffer):
                temp = (gridpoints[-2 - i][0], gridpoints[-2 - i][1])
                self.grid[self.to_index(temp[0], temp[1], self.size_x)] = 100
            temp = (gridpoints[-1][0], gridpoints[-1][1])
            self.grid[self.to_index(temp[0], temp[1], self.size_x)] = 100

        for i in range(len(points)):
            if points[i] == 0: continue
            rads = (radians(i) + cur_angle - math.pi/2) 
            endpos = self.to_grid(cur_pos[0] + (points[i]) * -math.sin(rads), cur_pos[1] + (points[i]) * math.cos(rads), self.origin_, self.res)
            endpos_prev = self.to_grid(cur_pos[0] + (points[(i - 1) %360]) * -math.sin(rads - math.pi/180), cur_pos[1] + (points[(i - 1) %360]) * math.cos(rads- math.pi/180), self.origin_, self.res)
            gridpoints = self.get_line(endpos, endpos_prev)
            if len(gridpoints) < 2 + self.ROBOT_WIDTH/self.res:
                for j in range(len(gridpoints)-1):
                    temp = (gridpoints[j][0], gridpoints[j][1])
                    self.grid[self.to_index(temp[0], temp[1], self.size_x)] = 100
        return

    def run(self):
        r = rospy.Rate(1)
        while not self.grid:
            r.sleep()
        print("size_x={size_x}, size_y={size_y}, grid size={grid_size}, origin_={pos}".format(size_x=self.size_x, size_y=self.size_y, grid_size=np.array(self.grid).size, pos=self.origin_))
        self.past_grids = []
        print(self.pos.x, self.pos.y, self.pos.theta)
        while not rospy.is_shutdown():
            self.past_grids.append(self.grid)
            self.map_all_lines()
            self.renew_objects()
            self.pub_occ_grid(self.past_grids)
            if len(self.past_grids) >= 6:
                self.past_grids = self.past_grids[1:]
            r.sleep()

    
if __name__ == '__main__':
    try:

        rospy.loginfo("You have reached the destination")
        thing = map_navigation()
        thing.run()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("map_navigation node terminated.")