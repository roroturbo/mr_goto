import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np
from skimage.segmentation import flood, flood_fill
from skimage import io, morphology, img_as_ubyte
#from graph import Graph, get_adjacency, display_adjacency
from typing import List, Tuple
import heapq

# To save map in the good file (Debug)
path_image_save = '/home/robot/Desktop/'

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('My node has started.')
        # Your code here
        #Topics & Subscriptions,Publishers
        map_topic = '/map'
        path_topic = '/path'
        position_topic = '/ground_truth'
        goal_topic = '/goal_pose'

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.map_callback, 1)
        self.pos_sub = self.create_subscription(Odometry, position_topic, self.odom_callback, 1)
        self.goal_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 1)
        self.path_pub = self.create_publisher(Path, path_topic, 10)

        self.postion = (0, 0, 0)
        self.map = [[]]

    def goal_callback(self, data):
        print("Update goal")
        position_x = data.pose.position.x
        position_y = data.pose.position.y
        orientation = data.pose.orientation

        map_binary = self.map

        # Get the starting point
        y, x = self.convertion(map_binary, self.postion[0], self.postion[1], True)
        y_goal, x_goal = self.convertion(map_binary, position_x, position_y, True)

        # Get the drivable_area
        map_binary = self.get_drivable_area(morphology.erosion(map_binary, morphology.square(10)), x, y)
        #self.save_map(map_binary, "map_drive.png")

        # Create a copy of the drivable area and erose for safe travel
        safe_area = morphology.erosion(map_binary, morphology.square(4))
        #self.save_map(safe_area, "map_safe.png")

        start_point = (x, y)
        end_point = (x_goal, y_goal)

        a_star = AStar()
        path = a_star.astar(safe_area, start_point, end_point)

        """for x in path:
            print(x," -> ", self.convertion(map_binary, x[0], x[1], False))"""

        self.publish_path(path, map_binary, 10)

    def odom_callback(self, data):
        #print("Update odom")
        position_x = data.pose.pose.position.x
        position_y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        self.postion = (position_x, position_y, orientation)
    
    def map_callback(self, data):
        """ 
            Process the map to pre-compute the path to follow and publish it
        """
        #Get_the_binary_map
        occupied_thresh = 0.65
        self.map = self.get_binary_map(data, occupied_thresh)
        #self.save_map(self.map, "map.png")

    def convertion(self, map : List[List[bool]], x, y, type: bool):
        """"
            Convert a point to the map coordinate and the image coordinate, in the both way (type)
        """
        size_image_x = 16
        size_image_y = 16

        scale_x = size_image_x / len(map)
        scale_y = - size_image_y / len(map[0])

        offset_x = -size_image_x /2
        offset_y = size_image_y /2

        matrix = np.array([[scale_x, 0, offset_x], [0, scale_y, offset_y], [0, 0, 1]])

        coord_map = [x, y, 1]
        
        if (type): # True -> map to image / False -> image to map 
            coord_world = coord_map * np.linalg.inv(matrix)
            return (int(np.sum(coord_world[0])), int(np.sum(coord_world[1])))
        else:
            coord_world = coord_map * matrix
            return (-np.sum(coord_world[0]), -np.sum(coord_world[1]))

    def save_map(self, map : List[List[bool]], name_file: str) -> None:
        """
            Save a 2D array of bool in an image
        """
        io.imsave(path_image_save + name_file, img_as_ubyte(map), check_contrast=False)

    def get_binary_map(self, data, occupied_thresh : float):
        """
            Get data from /map and create the appropriate value of each pixel, return the map as 2D Array of bool (False -> Obstacle, True -> Free)
        """
        map_data = np.asarray(data.data).reshape((data.info.width, data.info.height)) # parse map data into 2D numpy array
        map_normalized = map_data / np.amax(map_data.flatten()) # normalize map
        map_binary : List[List[bool]] = map_normalized < (occupied_thresh) # make binary occupancy map

        return np.flip(map_binary, 0) 
    
    def get_drivable_area(self, map : List[List[bool]], x, y):
        """
            Compute all pixel out-side of the circuit as obstacle
        """
        drivable_area = np.zeros((map.shape[0], map.shape[1]))

        for i in range(map.shape[0]):
            for j in range(map.shape[1]):
                if map[i][j] == True:
                    drivable_area[i][j] = 1
                else:
                    drivable_area[i][j] = 0

        # Flood fill the drivable area from the starting point.
        drivable_area = flood_fill(drivable_area, (x, y), 10)

        for i in range(drivable_area.shape[0]):
            for j in range(drivable_area.shape[1]):
                if drivable_area[i][j] == 10:
                    map[i][j] = True
                else:
                    map[i][j] = False

        return map
    
    def publish_path(self, path, map, n_sparse):
        """
        Publish the path in a the topic /path
        """
        path_msg = Path()
        time_stamp = Clock().now()
        path_msg.header.stamp = time_stamp.to_msg()  # Horodatage du message
        path_msg.header.frame_id = "world"
        if path is not None :
            for i, coord in enumerate(path):
                if i == 0 or i == len(path)-1:
                    pose = PoseStamped()

                    X, Y = self.convertion(map, float(coord[0]), float(coord[1]), False)
                    pose.pose.position.x = Y  # Coordonnée x
                    pose.pose.position.y = X  # Coordonnée y
                    pose.pose.position.z = 0.0  # Coordonnée z (0.0 pour 2D)

                    path_msg.poses.append(pose)
                elif i % n_sparse == 0:
                    pose = PoseStamped()

                    X, Y = self.convertion(map, float(coord[0]), float(coord[1]), False)
                    pose.pose.position.x = Y  # Coordonnée x
                    pose.pose.position.y = X  # Coordonnée y
                    pose.pose.position.z = 0.0  # Coordonnée z (0.0 pour 2D)

                    path_msg.poses.append(pose)
        else : 
            print("sah y a rien")

        self.path_pub.publish(path_msg)

class AStar:
    def heuristic(self, a, b):
        # Estimate the distance between two points
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    def astar(self, matrix, start_point, end_point):
        rows, cols = len(matrix), len(matrix[0])
        open_list = []
        closed_set = set()

        # Create an adjancecy matrix
        g = {start_point: 0}
        f = {start_point: self.heuristic(start_point, end_point)}
        parent = {}

        heapq.heappush(open_list, (f[start_point], start_point))

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == end_point:
                # Return the path find
                path = []
                while current in parent:
                    path.append(current)
                    current = parent[current]
                path.append(start_point)
                path.reverse()
                return path

            closed_set.add(current)

            # Schearch neighbor
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                neighbor = current[0] + dx, current[1] + dy

                if neighbor[0] < 0 or neighbor[0] >= rows or neighbor[1] < 0 or neighbor[1] >= cols:
                    continue

                if not matrix[neighbor[0]][neighbor[1]]:
                    continue

                neighbor_cost = g[current] + 1

                if neighbor in closed_set and neighbor_cost >= g.get(neighbor, 0):
                    continue

                if neighbor_cost < g.get(neighbor, 0) or neighbor not in [i[1] for i in open_list]:
                    parent[neighbor] = current
                    g[neighbor] = neighbor_cost
                    f[neighbor] = neighbor_cost + self.heuristic(neighbor, end_point)
                    heapq.heappush(open_list, (f[neighbor], neighbor))

        # If no path
        return None

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
