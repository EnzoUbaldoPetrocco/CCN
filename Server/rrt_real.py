"""
This module implements rrt strategies
"""

import os
from typing import List, Tuple
import math
import random
import time
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib
import copy

# Use a non-interactive backend to avoid threading issues
import matplotlib.pyplot as plt

MAP_FOLDER = "./uploaded_maps"


class Node:
    """ "
    class Representing a Node of the path
    """

    def __init__(self, x: float, y: float, parent = None, cost = float("inf")):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost

    def __truediv__(self, other):
        if isinstance(other, Node):
            return Node(self.x / other.x, self.y / other.y, parent=self.parent, cost=self.cost)
        elif isinstance(other, (int, float)):
            return Node(self.x / other, self.y / other, parent=self.parent, cost=self.cost)
        else:
            raise TypeError(
                f"Operand of operation / must be of types: [Node, int, float] not {type(other)}"
            )

    def __mul__(self, other):
        if isinstance(other, Node):
            return Node(self.x * other.x, self.y * other.y, parent=self.parent, cost=self.cost)
        elif isinstance(other, (int, float)):
            return Node(self.x * other, self.y * other, parent=self.parent, cost=self.cost)
        else:
            raise TypeError(
                f"Operand of operation / must be of types: [Node, int, float] not {type(other)}"
            )
        
    def __str__(self):
        # Called by print() or str()
        return f"Node: x: {self.x}, y: {self.y}, cost: {self.cost}"
    
    def __copy__(self):
        return Node(self.x, self.y)  


    def to_string(self) -> str:
        """
        Returns a string representation of the node.
        """
        return f"Node(x={self.x}, y={self.y}, cost={self.cost})"
    
    def __repr__(self):
        return self.__str__()
    
    def __add__(self, other):
        if isinstance(other, Node):
            return Node(self.x + other.x, self.y + other.y, parent=self.parent, cost=self.cost)
        elif isinstance(float, int):
            return Node(self.x + other, self.y + other, parent=self.parent, cost=self.cost)
        else:
            raise TypeError("Can only add Node, float, int to Node")

    def __sub__(self, other):
        if isinstance(other, Node):
            return Node(self.x - other.x, self.y - other.y, parent=self.parent, cost=self.cost)
        elif isinstance(float, int):
            return Node(self.x - other, self.y - other, parent=self.parent, cost=self.cost)
        else:
            raise TypeError("Can only subtract Node, float, int to Node")
        
class Obstacle(Node):
    """ "
    class inheriting from Node, this should represent an obstacle round
    """

    def __init__(self, x: float, y: float, radius: float, parent = None, cost = float("inf")):
        super().__init__(x, y, parent, cost)
        self.radius = radius

    def __truediv__(self, other):
        if isinstance(other, Obstacle):
            return Obstacle(self.x / other.x, self.y / other.y, self.radius / other.radius, parent=self.parent, cost=self.cost)
        elif isinstance(other, (int, float)):
            return Obstacle(self.x / other, self.y / other, self.radius / other, parent=self.parent, cost=self.cost)
        else:
            raise TypeError(
                f"Operand of operation / must be of types: [Obstacle, int, float] not {type(other)}"
            )

    def __mul__(self, other):
        if isinstance(other, Obstacle):
            return Obstacle(self.x * other.x, self.y * other.y, self.radius*other.radius, parent=self.parent, cost=self.cost)
        elif isinstance(other, (int, float)):
            return Obstacle(self.x * other, self.y * other, self.radius*other, parent=self.parent, cost=self.cost)
        else:
            raise TypeError(
                f"Operand of operation / must be of types: [Obstacle, int, float] not {type(other)}"
            )
        
    def __str__(self):
        return super().__str__() + f", radius: {self.radius}"
    
    def __repr__(self):
        return self.__str__()

    def to_string(self):
        return super().to_string() + f", radius={self.radius}"

    def __copy__(self):
        return Obstacle(self.x, self.y, self.radius, parent = self.parent, cost = self.cost)

class Human(Obstacle):
    """ "
    class inheriting from Obstacle, this should represent an obstacle with orientation
    """

    def __init__(self, x: float, y: float, radius: float, theta: float, parent = None, cost= float("inf")):
        super().__init__(x, y, radius, parent = parent, cost= cost)
        self.theta = theta

    def __truediv__(self, other):
        if isinstance(other, Human):
            return Human(self.x / other.x, self.y / other.y, self.radius / other.radius, self.theta, parent=self.parent, cost=self.cost)
        elif isinstance(other, (int, float)):
            return Human(self.x / other, self.y / other, self.radius / other, self.theta, parent=self.parent, cost=self.cost)
        else:
            raise TypeError(
                f"Operand of operation / must be of types: [Human, int, float] not {type(other)}"
            )

    def __mul__(self, other):
        if isinstance(other, Human):
            return Human(self.x * other.x, self.y * other.y, self.radius*other.radius, self.theta, parent=self.parent, cost=self.cost)
        elif isinstance(other, (int, float)):
            return Human(self.x * other, self.y * other, self.radius*other, self.theta, parent=self.parent, cost=self.cost)
        else:
            raise TypeError(
                f"Operand of operation / must be of types: [Human, int, float] not {type(other)}"
            )
    
    def __str__(self):
        return super().__str__() + f", theta: {self.theta}"
    
    def __repr__(self):
        return self.__str__()

    def to_string(self):
        return super().to_string() + f", theta={self.theta}"
    
    def __copy__(self):
        return Human(self.x, self.y, self.radius, self.theta, parent=self.parent, cost=self.cost)

    def __add__(self, other):
        if isinstance(other, Node):
            return Human(self.x + other.x, self.y + other.y, self.radius, self.theta, parent=self.parent, cost=self.cost)
        elif isinstance(float, int):
            return Human(self.x + other, self.y + other, self.radius, self.theta, parent=self.parent, cost=self.cost)
        else:
            raise TypeError("Can only add Node, float, int to Node")

    def __sub__(self, other):
        if isinstance(other, Node):
            return Human(self.x - other.x, self.y - other.y, self.radius, self.theta, parent=self.parent, cost=self.cost)
        elif isinstance(float, int):
            return Human(self.x - other, self.y - other, self.radius, self.theta, parent=self.parent, cost=self.cost)
        else:
            raise TypeError("Can only subtract Node, float, int to Node")
       

class RRT:
    """
    Class RRT embeds all the rrt related modules so that given a starting node, a goal node
    bounds of the map, the obstacles, a maximum iteration number, step size, radius of robot and
    tolerance can return path and smooth map
    """

    def __init__(
        self,
        start_node: Node,
        goal_node: Node,
        x_bounds: List[float],
        y_bounds: List[float],
        humans: List[Human],
        max_iterations: int,
        step_size: float,
        radius: float,
        tolerance: float,
        map_data: np.ndarray | str,
        map_resolution: float,
        map_origin: Tuple[float, float],
        obstacles: List[Obstacle] = None,
        debug: bool = False,
    ):
        self.debug = debug
        """
        Initializes the RRT algorithm with the given parameters.

        Args:
            start_node: The starting node of the path.
            goal_node: The goal node of the path.
            x_bounds: The bounds of the x-axis.
            y_bounds: The bounds of the y-axis.
            humans: A list of human obstacles.
            max_iterations: Maximum number of iterations for the RRT algorithm.
            step_size: Step size for extending nodes.
            radius: Radius of the robot or obstacle.
            tolerance: Tolerance for reaching the goal.
            map_data: The map representation (e.g., occupancy grid).
            map_resolution: Resolution of the map.
            map_origin: Origin point of the map.
            obstacles: A list of static obstacles (optional).
        """
        if obstacles is None:
            obstacles = []
        if type(map_data) == str:
            self.parse_map(map_data)
        else:
            self.map = map_data
        self.start_node = start_node
        self.goal_node = goal_node
        self.x_bounds = x_bounds
        self.y_bounds = y_bounds
        self.obstacles = obstacles
        self.max_iterations = max_iterations
        self.step_size = step_size
        self.radius = radius
        self.tolerance = tolerance
        self.humans = humans
        self.map_resolution = map_resolution
        self.map_origin = map_origin

        self.map_area = (x_bounds[1] - x_bounds[0]) * (y_bounds[1] - y_bounds[0])

    def parse_map(self, map_path: str) -> np.ndarray:
        """
        Parses a map from a file and returns it as a numpy array.
        Converts RGB to grayscale if needed, binarizes it, and flips Y axis.
        """
        map_png = plt.imread(os.path.join(MAP_FOLDER, map_path))
        print(f"Map shape: {map_png.shape}")

        if map_png.ndim == 3:
            # Convert RGB to grayscale if necessary
            map_png = np.mean(map_png, axis=2)

        # Normalize the map to binary values (0 for free space, 1 for obstacles)
        binary_map = (map_png < 0.75).astype(np.float32)

        # Flip vertically to invert the Y-axis
        #binary_map = np.flipud(binary_map)

        # Ensure binary format (0s and 1s)
        binary_map[binary_map > 0] = 1
        binary_map[binary_map <= 0] = 0

        self.map = binary_map
        return self.map

    def plot_map(self):
        """
        Plots the map using matplotlib.
        """
        plt.imshow(
            self.map,
            cmap="gray",
            origin="lower",
            extent=[
                self.x_bounds[0],
                self.x_bounds[1],
                self.y_bounds[0],
                self.y_bounds[1],
            ],
        )
        plt.title("Map")
        plt.xlabel("X")
        plt.show(block=True)
        plt.colorbar(label="Occupancy")
        plt.show()

    def get_plot_rrt_star_with_path(
        self,
        nodes: List[Node],
        start_node: Node,
        goal_node: Node,
        obstacles: List[Obstacle],
        smooth_path: List[Node],
        robot_node = Node(0,0)
    ) -> plt.Figure:
        """Plots the RRT* graph with the found path.

        Args:
            nodes: A list of nodes representing the RRT* graph.
            start_node: The starting node.
            goal_node: The goal node.
            obstacles: A list of obstacles.
            smooth_path: A list of nodes representing the smoothed path.
        """
        # Extract x and y coordinates of all nodes
        x_coords = [node.x for node in nodes]
        y_coords = [node.y for node in nodes]

        # Plot nodes and edges
        plt.figure(figsize=(8, 6))
        plt.scatter(x_coords, y_coords, color="blue", s=10)

        for i in range(len(nodes)):
            if nodes[i].parent is not None:
                plt.plot(
                    [nodes[i].x, nodes[i].parent.x],
                    [nodes[i].y, nodes[i].parent.y],
                    color="gray",
                    linewidth=0.5,
                )

        # Highlight start and end nodes
        plt.scatter(start_node.x, start_node.y, color="green", s=50, label="Start Node")
        plt.scatter(goal_node.x, goal_node.y, color="red", s=50, label="Goal Node")
        plt.scatter(robot_node.x, robot_node.y, color="yellow", s=50, label="Robot Node")
        

        # Plot the smoothed path
        if smooth_path:
            path_x = [node.x for node in smooth_path]
            path_y = [node.y for node in smooth_path]
            plt.plot(path_x, path_y, color="purple", linewidth=2, label="Path")

        plt.title("Informed RRT* Graph with Path")
        plt.xlabel("X")
        plt.ylabel("Y")

        # Plot obstacles
        for obstacle in obstacles:
            if isinstance(obstacle, Human):
                plt.gca().add_patch(
                    plt.Circle(
                        (obstacle.x, obstacle.y),
                        obstacle.radius,
                        color="red",
                        label="Human",
                    )
                )
            else:
                plt.gca().add_patch(
                    plt.Circle(
                        (obstacle.x, obstacle.y),
                        obstacle.radius,
                        color="black",
                        label="Obstacle",
                    )
                )

        # Plot the map as a background
        if hasattr(self, "map") and self.map is not None:
            plt.imshow(
                1 - self.map,
                cmap="gray",
                origin="lower",
                extent=[
                    self.x_bounds[0],
                    self.x_bounds[1],
                    self.y_bounds[0],
                    self.y_bounds[1],
                ],
                alpha=0.5,
            )

        plt.legend()
        plt.grid(True)
        fig = plt.gcf()
        return fig

    def distance(self, node1: Node, node2: Node) -> float:
        """
        Euclidian distance
        """
        return math.sqrt((node1.x - node2.x) ** 2 + (node1.y - node2.y) ** 2)

    def is_collision_free(self, node: Node) -> bool:
        # Map collision check
        i = int(node.x)
        j = int(node.y)

        if not (0 <= i < self.map.shape[1] and 0 <= j < self.map.shape[0]):
            return False  # Out of bounds
        if self.map[j, i] == 1:
            return False  # Map-based obstacle

        # Human proxemics check
        for human in self.humans:
            if self.distance(node, human) < (human.radius):
                return False  # Violates proxemics

        return True

    def nearest_neighbor(self, graph: Node, node: Node):
        """
        given a graph and the node, search for the nearest neighbor
        """
        min_distance = float("inf")
        nearest_node = None
        for existing_node in graph:
            dist = self.distance(node, existing_node)
            if dist < min_distance:
                min_distance = dist
                nearest_node = existing_node
        return nearest_node

    def steer(self, node1: Node, node2: Node, step_size: float) -> Node:
        """ "
        given the distance between two nodes, if the distance is less then
        the step size, it returns the second node, otherwise, returns a new node
        which has as distance the step size, and the orientation the direction
        between the nodes
        """
        dist = self.distance(node1, node2)
        if dist <= step_size:
            return node2
        else:
            angle = math.atan2(node2.y - node1.y, node2.x - node1.x)
            new_x = node1.x + step_size * math.cos(angle)
            new_y = node1.y + step_size * math.sin(angle)
            return Node(new_x, new_y)

    def find_neighbors(
        self, graph: List[Node], node: Node, radius: float
    ) -> List[Node]:
        """ "
        given a graph, a node and a radius, it returns the neighbors nearer
        then the radius
        """
        neighbors = []

        for existing_node in graph:
            if self.distance(node, existing_node) <= radius:
                neighbors.append(existing_node)
        return neighbors

    def human_proxemic_cost(self, node: Node, human: Human):
        dist = self.distance(node, human)
        
        if dist < human.radius:
            # Inside human radius: very high cost (collision)
            return float("inf")
        #elif human.radius <= dist <= human.radius + buffer:
            # Near boundary: **encourage** proximity by giving low cost
            # E.g. a decreasing linear function that is 0 at boundary and higher at boundary+buffer
        return (dist - human.radius)**2
        #else:
            # Far from human: neutral or slight penalty
        #    return 1.0

    def cost(self, neighbor, new_node):
        base_cost = neighbor.cost + self.distance(neighbor, new_node)
        proxemic_cost = 0.0
        for human in self.humans:
            proxemic_cost += self.human_proxemic_cost(new_node, human)
        proxemic_cost /= len(self.humans) if self.humans else 1
        # You can tune the weight of proxemic cost
        return base_cost + 5 * proxemic_cost

    def informed_rrt_star(self) -> List[Node]:
        """
        Informed RRT* with bias to sample near humans.
        """
        if not self.is_collision_free(self.start_node):
            # I want to start from the closest start_node that is not colliding
            # To do that I will take the start_node as the node that is not in the area of the human is colliding with
            # I can compute the line between the human and the start node and put the robot right outside that area.
            # Find the closest human the start_node is colliding with
            colliding_human = None
            for human in self.humans:
                if self.distance(self.start_node, human) < human.radius:
                    colliding_human = human
                    break
            if colliding_human is not None:
                # Compute direction from human to start_node
                dx = self.start_node.x - colliding_human.x
                dy = self.start_node.y - colliding_human.y
                norm = math.sqrt(dx**2 + dy**2)
                if norm == 0:
                    # If exactly at human center, pick arbitrary direction
                    dx, dy = 1.0, 0.0
                    norm = 1.0
                # Move just outside the human's radius (plus a small epsilon)
                epsilon = 1e-3
                new_x = colliding_human.x + (dx / norm) * (colliding_human.radius + epsilon)
                new_y = colliding_human.y + (dy / norm) * (colliding_human.radius + epsilon)
                self.start_node = Node(new_x, new_y)
            else:
                # If not colliding, keep original start_node
                pass
        graph = [self.start_node]
        
        self.start_node.cost = 0
        start_time = time.time()
        for _ in range(self.max_iterations):

            randx = random.uniform(self.x_bounds[0], self.x_bounds[1])
            randy = random.uniform(self.y_bounds[0], self.y_bounds[1])
            random_node = Node(randx, randy)

            if not self.is_collision_free(random_node):
                continue

            nearest_node = self.nearest_neighbor(graph, random_node)
            new_node = self.steer(nearest_node, random_node, self.step_size)
            if not self.is_collision_free(new_node):
                continue

            # Find neighbors and choose the best parent considering both cost and human_cost
            neighbors = self.find_neighbors(graph, new_node, self.radius)
            if not neighbors:
                continue
            best_parent = nearest_node
            best_cost = self.cost(
                nearest_node, new_node
            )  # nearest_node.cost + self.distance(nearest_node, new_node) #+ human_cost

            for neighbor in neighbors:
                # Calculate human cost for this neighbor as parent

                total_cost = self.cost(nearest_node, new_node)
                if total_cost < best_cost:
                    best_cost = total_cost
                    best_parent = neighbor

            new_node.cost = best_cost
            new_node.parent = best_parent

            graph.append(new_node)

            for neighbor in neighbors:
                if new_node.cost + self.distance(new_node, neighbor) < neighbor.cost:
                    neighbor.cost = new_node.cost + self.distance(new_node, neighbor)
                    neighbor.parent = new_node

        print(f"RRT* algorithm completed in: {time.time()-start_time}s.")
        print(f"Total nodes in graph: {len(graph)}")
        return graph

    def plot_rrt_star_with_path(
        self,
        nodes: List[Node],
        start_node: Node,
        goal_node: Node,
        smooth_path: List[Node],
    ):
        """Plots the RRT* graph with the found path.

        Args:
            nodes: A list of nodes representing the RRT* graph.
            start_node: The starting node.
            goal_node: The goal node.
            obstacles: A list of obstacles.
            smooth_path: A list of nodes representing the smoothed path.
        """
        # Extract x and y coordinates of all nodes
        x_coords = [node.x for node in nodes]
        y_coords = [node.y for node in nodes]

        # Plot nodes and edges
        plt.figure(figsize=(8, 6))
        plt.scatter(x_coords, y_coords, color="blue", s=10)

        for i in range(len(nodes)):
            if nodes[i].parent is not None:
                plt.plot(
                    [nodes[i].x, nodes[i].parent.x],
                    [nodes[i].y, nodes[i].parent.y],
                    color="gray",
                    linewidth=0.5,
                )

        # Highlight start and end nodes
        plt.scatter(start_node.x, start_node.y, color="green", s=50, label="Start Node")
        plt.scatter(goal_node.x, goal_node.y, color="red", s=50, label="Goal Node")

        # Plot the smoothed path
        if smooth_path:
            path_x = [node.x for node in smooth_path]
            path_y = [node.y for node in smooth_path]
            plt.plot(path_x, path_y, color="purple", linewidth=2, label="Path")

        plt.title("Informed RRT* Graph with Path")
        plt.xlabel("X")
        plt.ylabel("Y")

        # Plot obstacles
        for obstacle in self.obstacles:
            if isinstance(obstacle, Human):
                plt.gca().add_patch(
                    plt.Circle(
                        (obstacle.x, obstacle.y),
                        obstacle.radius,
                        color="red",
                        label="Human",
                    )
                )
            else:
                plt.gca().add_patch(
                    plt.Circle(
                        (obstacle.x, obstacle.y),
                        obstacle.radius,
                        color="black",
                        label="Obstacle",
                    )
                )

        plt.legend()
        plt.grid(True)
        plt.show()

    def plot_rrt_star_with_path_in_map(self, nodes, start_node, goal_node, smooth_path):
        plt.figure(figsize=(10, 8))

        # Draw map as image
        plt.imshow(
            1 - self.map,
            cmap="gray",
            origin="lower",
            extent=[
                self.x_bounds[0],
                self.x_bounds[1],
                self.y_bounds[0],
                self.y_bounds[1],
            ],
        )

        # Plot RRT graph
        for node in nodes:
            if node.parent is not None:
                plt.plot(
                    [node.x, node.parent.x],
                    [node.y, node.parent.y],
                    color="gray",
                    linewidth=0.5,
                )

        # Plot path
        if smooth_path:
            path_x = [node.x for node in smooth_path]
            path_y = [node.y for node in smooth_path]
            plt.plot(path_x, path_y, color="purple", linewidth=2, label="Path")

        # Start/goal
        plt.scatter(start_node.x, start_node.y, color="green", s=50, label="Start Node")
        plt.scatter(goal_node.x, goal_node.y, color="red", s=50, label="Goal Node")

        # Plot human proxemics
        for human in self.humans:
            plt.gca().add_patch(
                plt.Circle(
                    (human.x, human.y),
                    human.radius,
                    color="red",
                    alpha=0.4,
                    label="Human Zone",
                )
            )

        plt.legend()
        plt.grid(True)
        plt.title("RRT* with Human Proxemics and Map")
        plt.xlabel("X")
        plt.ylabel("Y")
        plt.show()

    def run_complete_and_get_plot(self, robot_node = Node(0,0)) -> List:
        """
        runs the complete algorithm until smooth path and plot the smooth
        """
        start = time.time()
        nodes = self.informed_rrt_star()
        optimal_path = self.extract_optimal_path(nodes, self.goal_node)
        smooth_path = self.interpolate_path(optimal_path)
        print(f"Total time taken: {time.time() - start:.2f} seconds")
        fig = self.get_plot_rrt_star_with_path(
            nodes, self.start_node, self.goal_node, self.obstacles, smooth_path, robot_node
        )
        return nodes, optimal_path, smooth_path, fig

    def extract_optimal_path(self, nodes: List[Node], goal_node: Node) -> List[Node]:
        """ "
        extract optimal path from nodes to goal node backward from goal node
        """
        # Find the closest node to the goal
        start = time.time()

        closest_node = min(nodes, key=lambda node: self.distance(node, goal_node))
        print(f"Closest node to goal found: {closest_node.to_string()}")

        # Backtrack to find the path
        path = []
        current_node = closest_node
        while current_node is not None:
            path.append(current_node)
            current_node = current_node.parent  # Move to the parent node
            # print(f"Current node in path: {current_node.to_string() if current_node else 'None'}")

        # Reverse to get path from start to goal
        path.reverse()
        print(f"Optimal path extracted in: {time.time() - start:.2f} seconds.")
        return path

    def interpolate_path(self, path: List[Node]) -> List[Node]:
        """Interpolate a smooth path using cubic splines."""
        start = time.time()
        if len(path) < 3:
            return path  # No need to interpolate small paths

        x = [node.x for node in path]
        y = [node.y for node in path]

        # Create a cubic spline function
        t = np.linspace(0, 1, len(path))  # Normalize parameter
        spline_x = CubicSpline(t, x)
        spline_y = CubicSpline(t, y)

        # Generate new smooth points
        t_smooth = np.linspace(0, 1, 3 * len(path))  # More points for smoothness
        smooth_x = spline_x(t_smooth)
        smooth_y = spline_y(t_smooth)

        # Convert back to nodes
        smooth_path = [Node(x, y) for x, y in zip(smooth_x, smooth_y)]
        print(f"Path interpolation completed in: {time.time() - start:.2f} seconds.")
        return smooth_path

    def run_complete(self) -> List[List[Node]]:
        """
        runs the complete algorithm until smooth path
        """
        print("Running informed RRT* algorithm...")
        nodes = self.informed_rrt_star()
        print("Extracting optimal path...")
        optimal_path = self.extract_optimal_path(nodes, self.goal_node)
        print("Interpolating path for smoothness...")
        smooth_path = self.interpolate_path(optimal_path)
        print("Path extraction and interpolation complete.")
        return nodes, optimal_path, smooth_path

    def run_complete_and_plot(self) -> List[List[Node]]:
        """
        runs the complete algorithm until smooth path and plot the smooth
        """
        start = time.time()
        print("Running informed RRT* algorithm...")
        nodes = self.informed_rrt_star()
        print("Extracting optimal path...")
        optimal_path = self.extract_optimal_path(nodes, self.goal_node)
        print("Interpolating path for smoothness...")
        smooth_path = self.interpolate_path(optimal_path)
        print("Path extraction and interpolation complete.")
        print(f"Total time taken: {time.time() - start:.2f} seconds")
        self.plot_rrt_star_with_path(
            nodes, self.start_node, self.goal_node, smooth_path
        )
        return nodes, optimal_path, smooth_path


def main():
    """
    Runs an example for testing RRT algorithm
    """
    # Example usage:
    start_node = Node(3.5, -3.5)
    goal_node = Node(-3.5, 3.5)
    obstacles = [Obstacle(2, 2, 0.5), Human(0, 0, 0.5, 0.5)]
    max_iterations = 2500
    step_size = 0.8
    radius = 0.7
    tolerance = 0.2
    x_bounds, y_bounds = (-5, 5), (-5, 5)

    # Dummy/default values for map_data, map_resolution, and map_origin for testing
    map_data = np.zeros((100, 100))  # Example: empty map
    map_resolution = 0.1  # Example: 0.1 units per pixel
    map_origin = (x_bounds[0], y_bounds[0])  # Example: origin at lower bounds

    rrt = RRT(
        start_node,
        goal_node,
        x_bounds,
        y_bounds,
        [ob for ob in obstacles if isinstance(ob, Human)],  # humans
        max_iterations,
        step_size,
        radius,
        tolerance,
        map_data,
        map_resolution,
        map_origin,
        [ob for ob in obstacles if not isinstance(ob, Human)],  # obstacles
    )

    nodes = rrt.informed_rrt_star()

    # Assuming 'nodes' is the list of nodes generated by informed_rrt_star

    # Extract x and y coordinates of all nodes
    x_coords = [node.x for node in nodes]
    y_coords = [node.y for node in nodes]

    # Plot nodes and edges
    plt.figure(figsize=(8, 6))
    plt.scatter(x_coords, y_coords, color="blue", s=10)

    for i in range(len(nodes)):
        if nodes[i].parent is not None:
            plt.plot(
                [nodes[i].x, nodes[i].parent.x],
                [nodes[i].y, nodes[i].parent.y],
                color="gray",
                linewidth=0.5,
            )

    # Highlight start and end nodes
    plt.scatter(start_node.x, start_node.y, color="green", s=50, label="Start Node")
    plt.scatter(goal_node.x, goal_node.y, color="red", s=50, label="Goal Node")

    # Highlight last node found (assuming last node is the closest to the goal)

    last_node_found = nodes[-1]
    plt.scatter(
        last_node_found.x,
        last_node_found.y,
        color="purple",
        s=30,
        label="Last Node Found",
    )
    plt.title("Informed RRT* Graph")

    plt.xlabel("X")
    plt.ylabel("Y")

    # Plot obstacles
    for obstacle in obstacles:
        if isinstance(obstacle, Human):
            plt.gca().add_patch(
                plt.Circle(
                    (obstacle.x, obstacle.y),
                    obstacle.radius,
                    color="red",
                    label="Human",
                )
            )
        else:
            plt.gca().add_patch(
                plt.Circle(
                    (obstacle.x, obstacle.y),
                    obstacle.radius,
                    color="black",
                    label="Obstacle",
                )
            )

    plt.legend()
    plt.grid(True)
    plt.show()

    optimal_path = rrt.extract_optimal_path(nodes, goal_node)
    rrt.plot_rrt_star_with_path(nodes, start_node, goal_node, optimal_path)

    smooth_path = rrt.interpolate_path(optimal_path)
    rrt.plot_rrt_star_with_path(nodes, start_node, goal_node, smooth_path)

    # Measuring performance of the rrt until interpolation
    num_trials = 150
    total_time = 0.0

    for _ in range(num_trials):
        start_time = time.time()
        nodes = rrt.informed_rrt_star()
        optimal_path = rrt.extract_optimal_path(nodes, goal_node)
        smooth_path = rrt.interpolate_path(optimal_path)
        end_time = time.time()
        total_time += end_time - start_time

    average_time = total_time / num_trials
    print(f"Average time to find a smooth path: {average_time:.4f} seconds")

    for proximity in np.linspace(0.3, 1.8, 21):
        obstacles = [Human(0, 0, proximity, 0.5)]
        nodes = rrt.informed_rrt_star()
        optimal_path = rrt.extract_optimal_path(nodes, goal_node)
        smooth_path = rrt.interpolate_path(optimal_path)
        rrt.plot_rrt_star_with_path(nodes, start_node, goal_node, smooth_path)

    x_bounds = (0, 3)
    y_bounds = (0, 4)
    start_node = Node(0, 1)
    goal_node = Node(2.7, 3.7)
    for proximity in np.linspace(0.46, 2.1, 8):
        obstacles = [Human(0, 2.5, proximity, 0.5)]
        nodes = rrt.informed_rrt_star()
        optimal_path = rrt.extract_optimal_path(nodes, goal_node)
        smooth_path = rrt.interpolate_path(optimal_path)
        rrt.plot_rrt_star_with_path(nodes, start_node, goal_node, smooth_path)


if __name__ == "__main__":
    main()
