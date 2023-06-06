import rclpy
from rclpy.node import Node
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
        self.map_sub = self.create_subscription(Odometry, position_topic, self.odom_callback, 1)
        self.map_sub = self.create_subscription(PoseStamped, goal_topic, self.goal_callback, 1)
        self.path_pub = self.create_publisher(Path, path_topic, 10)


        self.postion = (0, 0, 0)
        self.goal = (0, 0, 0)

    def goal_callback(self, data):
        #print("Update goal")
        position_x = data.pose.position.x
        position_y = data.pose.position.y
        orientation = data.pose.orientation
        self.goal = (position_x, position_y, orientation)

    def odom_callback(self, data):
        #print("Update odom")
        position_x = data.pose.pose.position.x
        position_y = data.pose.pose.position.y
        orientation = data.pose.pose.orientation
        self.postion = (position_x, position_y, orientation)

        rate = self.create_rate(10)  # Vitesse de publication (10 Hz dans cet exemple)

        while not rclpy.ok():
            print("path")
            path_msg = Path()
            path_msg.header.stamp = rclpy.Time.now()  # Horodatage du message
            path_msg.header.frame_id = "map"

            pose1 = PoseStamped()
            pose1.pose.position.x = -7  # Coordonnée x
            pose1.pose.position.y = -7  # Coordonnée y
            pose1.pose.position.z = 0.0  # Coordonnée z (0.0 pour 2D)

            pose2 = PoseStamped()
            pose2.pose.position.x = 0  # Coordonnée x
            pose2.pose.position.y = 7  # Coordonnée y
            pose2.pose.position.z = 0.0  # Coordonnée z (0.0 pour 2D)

            path_msg.poses.append(pose1)
            path_msg.poses.append(pose2)

            self.path_pub.publish(path_msg)
            rate.sleep()
    
    def map_callback(self, data):
        """ 
            Process the map to pre-compute the path to follow and publish it
        """
        #Get_the_binary_map
        print("Start")
        occupied_thresh = 0.65
        map_binary = self.get_binary_map(data, occupied_thresh)
        self.save_map(map_binary, "map.png")

        # Get the starting point.
        x, y = self.convertion(map_binary, self.postion[0], self.postion[1], True)
        x_goal, y_goal = self.convertion(map_binary, -6, -5, True)

        # Get the drivable_area
        print("X = ", x)
        print("Y = ", y)
        print("X = ", x_goal)
        print("Y = ", y_goal)
        print("map erode")
        map_binary = self.get_drivable_area(morphology.erosion(map_binary, morphology.square(2)), x, y)
        self.save_map(map_binary, "map_drive.png")


        # Create a copy of the drivable area and erose for safe travel
        print("map safe")
        safe_area = morphology.erosion(map_binary, morphology.square(4))
        self.save_map(safe_area, "map_safe.png")

        start_point = (x, y)
        end_point = (x_goal, y_goal)

        a_star = AStar()
        path = a_star.astar(safe_area, start_point, end_point)
        print("map_publidh")

        for x in path:
            print(x)

        """# Define the final point
        adjacency_list = graph.get_adjacency(safe_area)
        graph = Graph(adjacency_list)

        start_node = str(x)+','+str(y)
        stop_node = str(x_goal)+','+str(y_goal)
        print("path create")
        path = graph.a_star_algorithm(start_node, stop_node)
        print("path publish")"""
        #self.publish_path(path, map_binary)
        print("finish")

    def convertion(self, map : List[List[bool]], x, y, type: bool):
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
            return (np.sum(coord_world[0]), np.sum(coord_world[1]))

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
    
    def publish_path(self, path, map):
        """
        Publish the path in a the topic /path
        """
        rate = self.create_rate(10)  # Vitesse de publication (10 Hz dans cet exemple)

        while not rclpy.ok():
            path_msg = Path()
            path_msg.header.stamp = rclpy.Time.now()  # Horodatage du message
            path_msg.header.frame_id = "map"

            for coord in path:
                pose = PoseStamped()

                X, Y = self.convertion(map, float(coord[0]), float(coord[1]), False)
                pose.pose.position.x = X  # Coordonnée x
                pose.pose.position.y = Y  # Coordonnée y
                pose.pose.position.z = 0.0  # Coordonnée z (0.0 pour 2D)

                path_msg.poses.append(pose)

            self.path_pub.publish(path_msg)
            rate.sleep()



class Graph:
    def __init__(self, adjacency_list):
        self.adjacency_list = adjacency_list

    def get_neighbors(self, v):
        return self.adjacency_list[v]

    def h(self, n):
        H = {}
        for node in self.adjacency_list:
            H[node] = 1
        return H[n]

    def a_star_algorithm(self, start_node, stop_node):
        open_list = set([start_node])
        closed_list = set([])

        g = {}
        g[start_node] = 0

        parents = {}
        parents[start_node] = start_node

        while len(open_list) > 0:
            n = None

            for v in open_list:
                if n == None or g[v] + self.h(v) < g[n] + self.h(n):
                    n = v

            if n == None:
                print('Path does not exist!')
                return None

            if n == stop_node:
                reconst_path = []
                while parents[n] != n:
                    reconst_path.append(n)
                    n = parents[n]
                reconst_path.append(start_node)
                reconst_path.reverse()
                #print('Path found: {}'.format(reconst_path))
                return reconst_path

            for (m, weight) in self.get_neighbors(n):
                if m not in open_list and m not in closed_list:
                    open_list.add(m)
                    parents[m] = n
                    g[m] = g[n] + weight
                else:
                    if g[m] > g[n] + weight:
                        g[m] = g[n] + weight
                        parents[m] = n
                        if m in closed_list:
                            closed_list.remove(m)
                            open_list.add(m)

            open_list.remove(n)
            closed_list.add(n)

        print('Path does not exist!')
        return None

    def get_adjacency(matrix):
        adjacency_list = {}
        n = len(matrix)

        for i in range(n):
            for j in range(n):
                if matrix[i][j]:
                    neighbors = []
                    if i > 0 and matrix[i - 1][j]:
                        neighbors.append((f'{i - 1},{j}', 1))
                    if i < n - 1 and matrix[i + 1][j]:
                        neighbors.append((f'{i + 1},{j}', 1))
                    if j > 0 and matrix[i][j - 1]:
                        neighbors.append((f'{i},{j - 1}', 1))
                    if j < n - 1 and matrix[i][j + 1]:
                        neighbors.append((f'{i},{j + 1}', 1))
                    adjacency_list[f'{i},{j}'] = neighbors

        return adjacency_list
    
    def display_adjacency(graph):
        adjacency_list = graph.adjacency_list

        for node, neighbors in adjacency_list.items():
            print(f"Voisins du nœud {node}:")
            for neighbor, weight in neighbors:
                print(f"- {neighbor} (poids: {weight})")
            print()

class AStar:
    def heuristic(self, a, b):
        # Heuristique utilisée pour estimer la distance entre deux points (ici, distance de Manhattan)
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    def astar(self, matrix, start_point, end_point):
        rows, cols = len(matrix), len(matrix[0])
        open_list = []
        closed_set = set()

        # Création d'un dictionnaire pour stocker le coût total du chemin et le parent de chaque point
        g = {start_point: 0}
        f = {start_point: self.heuristic(start_point, end_point)}
        parent = {}

        heapq.heappush(open_list, (f[start_point], start_point))

        while open_list:
            current = heapq.heappop(open_list)[1]

            if current == end_point:
                # Retourne le chemin trouvé
                path = []
                while current in parent:
                    path.append(current)
                    current = parent[current]
                path.append(start_point)
                path.reverse()
                return path

            closed_set.add(current)

            # Recherche des voisins du point courant
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

        # Si aucun chemin n'a été trouvé
        return None

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
