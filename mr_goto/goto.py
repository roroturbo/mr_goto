import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import numpy as np
from skimage.segmentation import flood, flood_fill
from skimage import io, morphology, img_as_ubyte
from graph import Graph, get_adjacency, display_adjacency
from typing import List, Tuple
from geometry_msgs.msg import PoseStamped




class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        self.get_logger().info('My node has started.')
        # Your code here
        #Topics & Subscriptions,Publishers
        map_topic = '/map'
        path_topic = '/path'

        self.map_sub = self.create_subscription(map_topic, OccupancyGrid, self.map_callback, queue_size=1)
        self.path_pub = self.create_publisher(path_topic, Path, queue_size=10)
    
    def map_callback(self, data):
        """ 
            Process the map to pre-compute the path to follow and publish it
        """

        # Get the starting point.
        #starting_point = data.info.origin
        x : int = 1000
        y : int = 1000

        #Get_the_binary_map
        occupied_thresh = 0.65
        map_binary = self.get_binary_map(data, occupied_thresh)

        # Get the drivable_area
        map_binary = self.get_drivable_area(map_binary, x, y)

        # Create a copy of the drivable area and erose for safe travel
        eroded_drivable_area = np.copy(map_binary)
        safe_area = morphology.erosion(eroded_drivable_area, morphology.square(4))

        # Define the final point
        adjacency_list = get_adjacency(safe_area)
        graph = Graph(adjacency_list)

        start_node = '1000,1001'
        stop_node = '1000,999'
        path = graph.a_star_algorithm(start_node, stop_node)
        self.publish_path(path)
    
    def get_binary_map(self, data, occupied_thresh : float):
        """
            Get data from /map and create the appropriate value of each pixel, return the map as 2D Array of bool (False -> Obstacle, True -> Free)
        """
        map_data = np.asarray(data.data).reshape((data.info.width, data.info.height)) # parse map data into 2D numpy array
        map_normalized = map_data / np.amax(map_data.flatten()) # normalize map
        map_binary : List[List[bool]] = map_normalized < (occupied_thresh) # make binary occupancy map

        return map_binary
    
    def get_drivable_area(self, map : List[List[bool]], x : int, y : int):
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
    
    def publish_path(self, path):
        """
            Publish the path in a the topic /path
        """
        rate = rclpy.Rate(10)  # Vitesse de publication (10 Hz dans cet exemple)

        while not rclpy.is_shutdown():
            path_msg = Path()
            path_msg.header.stamp = rclpy.Time.now()  # Horodatage du message
            path_msg.header.frame_id = "map"

            for coord in path:
                pose = PoseStamped()

                X = float(coord.split(',')[0])
                Y = float(coord.split(',')[1])
                pose.pose.position.x = (Y - 1000) / 20  # Coordonnée x
                pose.pose.position.y = (X - 1000) / 20 # Coordonnée y
                pose.pose.position.z = 0.0  # Coordonnée z (0.0 pour 2D)

                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            rate.sleep()




def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
