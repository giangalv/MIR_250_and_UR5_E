import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener, LookupException
from tf_transformations import euler_from_quaternion
from nav2_dynamic_reconfig import Nav2DynamicReconfig

import yaml
import math
import networkx as nx
import os


def euclidean_distance(x1, y1, x2, y2):
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


class GraphNavController(Node):
    def __init__(self):
        super().__init__('graph_nav_controller')

        # ===== Configuration =====
        self.PACKAGE_NAME = 'graph_based_navigation_system'
        self.GRAPH_NAME = 'navigation_graph_2'  # Base name without extension

        # Default velocity per style
        self.style_velocity_map = {
            "default": 0.3,
            "fast": 0.7,
            "slow": 0.15,
            "cautious": 0.2
        }

        # ===== Path Setup =====
        self.graph_file = self.get_graph_path()
        self.load_graph()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_config = Nav2DynamicReconfig(self)  # For velocity changes

        self.timer = self.create_timer(2.0, self.prompt_user)

    def get_graph_path(self):
        """Get graph file path with fallback to install directory"""
        src_dir = os.path.join(
            get_package_share_directory(self.PACKAGE_NAME),
            '../..', 'src', self.PACKAGE_NAME, 'config'
        )
        src_path = os.path.join(src_dir, f'{self.GRAPH_NAME}.yaml')

        if os.path.exists(src_path):
            return src_path

        install_dir = os.path.join(
            get_package_share_directory(self.PACKAGE_NAME),
            'config'
        )
        install_path = os.path.join(install_dir, f'{self.GRAPH_NAME}.yaml')

        if os.path.exists(install_path):
            return install_path

        raise FileNotFoundError(
            f"Graph file {self.GRAPH_NAME}.yaml not found in either:\n"
            f"- {src_dir}\n"
            f"- {install_dir}"
        )
    
    def get_path(self, start_node, goal_node):
        planner = self.nodes[goal_node].get('planner', 'dijkstra')

        if planner == 'astar':
            self.get_logger().info(f"Using A* to compute path to node {goal_node}.")
            return nx.astar_path(
                self.G,
                source=start_node,
                target=goal_node,
                heuristic=lambda u, v: euclidean_distance(
                    self.nodes[u]['x'], self.nodes[u]['y'],
                    self.nodes[v]['x'], self.nodes[v]['y']
                ),
                weight='weight'
            )

        elif planner == 'bellman':
            self.get_logger().info(f"Using Bellman-Ford to compute path to node {goal_node}.")
            return nx.bellman_ford_path(self.G, source=start_node, target=goal_node, weight='weight')

        elif planner == 'dijkstra':
            self.get_logger().info(f"Using Dijkstra to compute path to node {goal_node}.")
            return nx.shortest_path(self.G, source=start_node, target=goal_node, weight='weight')

        else:
            self.get_logger().warn(f"Unknown planner '{planner}' for node {goal_node}. Defaulting to Dijkstra.")
            return nx.shortest_path(self.G, source=start_node, target=goal_node, weight='weight')
        
    def load_graph(self):
        with open(self.graph_file, 'r') as f:
            data = yaml.safe_load(f)

        # Convert node IDs from str to int
        self.nodes = {int(k): v for k, v in data['nodes'].items()}

        self.G = nx.Graph()
        for node_id, props in self.nodes.items():
            self.G.add_node(node_id, **props)

        for edge in data['edges']:
            weight = edge.get('weight', 1.0)
            self.G.add_edge(edge['from'], edge['to'], weight=weight)

        self.get_logger().info(f"Loaded navigation graph from: {self.graph_file}")

    def get_closest_node(self, current_x, current_y):
        min_dist = float('inf')
        closest = None
        for node_id, props in self.nodes.items():
            dist = euclidean_distance(current_x, current_y, props['x'], props['y'])
            if dist < min_dist:
                min_dist = dist
                closest = node_id
        return closest

    def prompt_user(self):
        self.timer.cancel()

        x, y = self.get_robot_position_xy()
        if x is not None and y is not None:
            self.get_logger().info(f"Robot position: x={x:.2f}, y={y:.2f}")
        else:
            print("[ERROR] Could not get robot pose.")
            return

        start_node = self.get_closest_node(x, y)
        print(f"\n[INFO] Robot is currently closest to node {start_node}.")

        while True:
            try:
                goal_node = int(input("Enter the goal node ID to navigate to (or -1 to exit): "))
            except ValueError:
                print("[ERROR] Invalid input. Please enter a number.")
                continue

            if goal_node == -1:
                print("[INFO] Exiting navigation loop.")
                break

            if goal_node not in self.nodes:
                print(f"[ERROR] Node {goal_node} does not exist in the graph.")
                continue

            try:
                path = self.get_path(start_node, goal_node)
            except nx.NetworkXNoPath:
                print(f"[ERROR] No path exists from node {start_node} to node {goal_node}.")
                continue

            print(f"[INFO] Planned path: {path}")
            self.execute_path(path)

            start_node = goal_node

    def get_robot_position_xy(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', now
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f"Failed to get robot position: {e}")
            return None, None

    def set_velocity_by_style(self, style):
        """Adjust Nav2 max velocity based on style."""
        if style not in self.style_velocity_map:
            style = "default"
        velocity = self.style_velocity_map[style]
        self.nav2_config.set_max_velocity(velocity)
        print(f"[CONFIG] Style '{style}' → max velocity set to {velocity} m/s")

    def execute_path(self, path):
        for node_id in path[1:]:
            props = self.nodes[node_id]

            # Apply velocity based on style
            style = props.get('style', 'default')
            self.set_velocity_by_style(style)

            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = props['x']
            goal.pose.position.y = props['y']
            goal.pose.orientation.z = math.sin(props['yaw'] / 2)
            goal.pose.orientation.w = math.cos(props['yaw'] / 2)

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal

            print(f"[NAV] Navigating to node {node_id} (style: {style})...")

            self.nav_client.wait_for_server()
            send_goal_future = self.nav_client.send_goal_async(nav_goal)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                print(f"[ERROR] Goal to node {node_id} was rejected by the server.")
                return

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result().result

            if result.error_code == 0:
                print(f"[SUCCESS] Reached node {node_id} successfully!\n")
            else:
                print(f"[FAILURE] Failed to reach node {node_id}. Error code: {result.error_code}\n")
                break
        print(f"[INFO] Completed full path to goal node {path[-1]}!\n")


def main():
    rclpy.init()
    node = GraphNavController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
