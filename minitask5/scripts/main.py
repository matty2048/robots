import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
from minitask5.msg import image_proc, object_data
import tf
import math

class Position:
    def __init__(self,x,y,theta):
        self.x : float = x 
        self.y : float = y
        self.theta : float = theta

class Main:
    SLEEP_RATE = 1

    def __init__(self):
        # Parameter initialisation
        self.param_max_green_boxes = rospy.get_param('/main/max_green_boxes')
        self.param_max_red_hydrants = rospy.get_param('/main/max_red_hydrants')
        self.red_tolerance = rospy.get_param('/main/red_tolerance')
        self.green_tolerance = rospy.get_param('/main/green_tolerance')
        self.red_radius = rospy.get_param('/main/red_radius')
        self.green_radius = rospy.get_param('/main/green_radius')
        self.obj_detection_range = rospy.get_param('/main/detection_range')

        # Variable initialisation
        self.objects_found: object_data = []
        self.red_obj = []
        self.green_obj = []
        self.pos = Position(0, 0, 0)

        # Grid variable initialisation
        self.resolution = rospy.get_param('/mapping/res')
        self.width = rospy.get_param('/mapping/size_x')
        self.height = rospy.get_param('/mapping/size_y')
        self.corr_x = rospy.get_param('/mapping/corr_x')
        self.origin = (-10, -10)
        self.grid = [-1]*self.width*self.height

        # Publisher and Subcriber initialisation
        # rospy.Subscriber('scan', LaserScan, self.callback_laser)
        rospy.Subscriber('map', OccupancyGrid, self.callback_occ)
        rospy.Subscriber('/image_proc', image_proc, self.callback_objects)
        rospy.Subscriber('odom', Odometry, self.callback_odom)

        # Create main node
        rospy.init_node('main', anonymous=False)

        # Set up shutdown callback
        self.node_name= rospy.get_name()
        rospy.on_shutdown(self.callback_shutdown) 

    def callback_objects(self, msg: image_proc):
        self.objects_found = msg.object_data

    def callback_occ(self, msg: OccupancyGrid):
        self.grid = msg.data
        self.resolution = msg.info.resolution
        self.width = msg.info.width
        self.height = msg.info.height
        self.origin = (msg.info.origin.position.x, msg.info.origin.position.y)

    def callback_odom(self, msg):
        quarternion = [msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pos.theta = yaw
        self.pos.x = msg.pose.pose.position.x
        self.pos.y = msg.pose.pose.position.y

    # Goes through all objects received from /image_proc and checks if they are duplicate objects and valid objects 
    def add_objects_to_list(self):
        for obj in self.objects_found:
            obj: object_data
            # Checks obj colour and if too many objects have been added to list
            # Checks if it has already been added
            # Checks if it's in the map
            # Checks if it's close enough to the robot to be counted
            if obj.red > 200 and len(self.red_obj) < 3*self.param_max_red_hydrants:
                if self.check_obj_in_list(obj, self.red_obj, self.red_tolerance): continue
                if not self.check_obj_in_map(obj, self.grid, int(self.red_radius/self.resolution)): continue
                if self.check_obj_dist_to_robot(obj, self.pos) > self.obj_detection_range: continue
                self.red_obj.append(obj)
            if obj.green > 200 and len(self.green_obj) < 3*self.param_max_green_boxes:
                if self.check_obj_in_list(obj, self.green_obj, self.green_tolerance): continue
                if not self.check_obj_in_map(obj, self.grid, int(self.green_radius/self.resolution)): continue
                if self.check_obj_dist_to_robot(obj, self.pos) > self.obj_detection_range: continue
                self.green_obj.append(obj)
    
    def check_dist_between(self, obj: object_data, obj2: object_data):
        return math.dist((obj.x_location, obj.y_location), (obj2.x_location, obj2.y_location))

    # Checks if the object is already within the list within a given distance tolerance (m)
    # Returns true if the list is exhausted
    # Returns false if an object is found
    def check_obj_in_list(self, obj: object_data, obj_list: object_data, tolerance):
        if len(obj_list) == 0: return False
        else:
            for i in range(len(obj_list)):
                if self.check_dist_between(obj, obj_list[i]) < tolerance: return False
                elif i == len(obj_list) - 1 : return True
    
    # Checks if the object is already within the list within a given radius from it's stated location (m/res)
    # Returns true if the object is in the map
    # Returns false if an object is not found in the map
    def check_obj_in_map(self, obj: object_data, map_data, radius):
        coords = self.to_grid(obj.x_location, obj.y_location, self.origin, self.resolution)
        size_r = radius
        count = 0
        for i in range(-size_r, size_r):
            for j in range(-size_r, size_r):
                if map_data[self.to_index(coords[0] + i, coords[1] + j, self.width)] > 65: count += 1
        # Compares the ratio of tiles discovered to undiscovered in that region
        if count / math.pow(size_r, 2) > 0.1: return True
        else: return False

    def check_obj_dist_to_robot(self, obj:object_data, pos: Position):
        return math.dist((obj.x_location, obj.y_location), (pos.x, pos.y))

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

    # Function to run when the node is shutdown
    # Prints out the objects that it found during it's exploration of the area
    def callback_shutdown(self):
        print(self.node_name + " shutting down")
        print("Found {r_len} number of red objects and {g_len} number of green objects".format(r_len= len(self.red_obj), g_len= len(self.green_obj)))
        if len(self.red_obj) >= 2 * self.param_max_red_hydrants:
            print("Found too many red objects to print")
        else: print(self.red_obj)
        if len(self.green_obj) >= 2 * self.param_max_green_boxes:
            print("Found too many green objects to print")
        else: print(self.green_obj)

    def run(self):
        r = rospy.Rate(self.SLEEP_RATE)
        # Flags for printing when it's found the correct number of objects
        found_red = False
        found_green = False
        while not rospy.is_shutdown():
            self.add_objects_to_list()

            if len(self.red_obj) == self.param_max_red_hydrants and not found_red: 
                print("Found all red objects at locations:")
                print(self.red_obj)
                found_red = True
            
            if len(self.green_obj) == self.param_max_green_boxes and not found_green:
                print("Found all green objects at locations:")
                print(self.green_obj)
                found_green = True
            
            if len(self.red_obj) > 2*self.param_max_red_hydrants or len(self.green_obj) > 2*self.param_max_green_boxes:
                print("Found too many red or green objects, shutting down node safely")
                break
            r.sleep()
            

if __name__ == '__main__':
    try:
        main = Main()
        main.run()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass