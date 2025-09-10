#!/usr/bin/env python3
import os
import math
import time
import yaml
import networkx as nx

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from geometry_msgs.msg import Quaternion
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

from tf2_ros import Buffer, TransformListener

from ament_index_python.packages import get_package_share_directory

# NOTE: This import assumes you have these helper classes available as in your repo.
# - NavigationDefaults: holds defaults for nodes/edges and style profiles
# - Nav2DynamicReconfig: small wrapper to set params on Nav2 nodes
from graph_based_navigation_system.navigation_defaults import NavigationDefaults
from .nav2_dynamic_reconfig import Nav2DynamicReconfig

_GRAPH_NAME = 'test_TER_1'  # Default graph name, can be overridden

def euclidean_distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)

def yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (in radians) into a Quaternion message."""
    q = Quaternion()
    q.w = math.cos(yaw / 2.0)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    return q

class GraphNavController(Node):
    def __init__(self,):
        super().__init__('graph_nav_controller')

        self.GRAPH_NAME = f'{_GRAPH_NAME}_graph.yaml'
        # From navigation_defaults.py load defaults navigation parameters
        self.defaults = NavigationDefaults()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # internal state
        self.nodes = {}
        self.edges = {}
        self.DG = nx.DiGraph()

        # load graph
        self.graph_file = self.get_graph_path()
        self.load_graph()

        self.get_logger().info('GraphNavController initialized and ready.')

        # subscriber
        self.pose_received = False
        self.mir_pose = None
        self.robot_position_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )

        # small delay to let TF and nav2 come up
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateToPose server not available yet. Will wait when sending goals.')

        # start the interactive prompt after short delay to ensure node finishes init
        # use a timer so rclpy spin can run concurrently
        self.create_timer(0.1, self.prompt_user)

    # ----------------------
    # File & Graph helpers
    # ----------------------
    def get_graph_path(self):
        pkg_dir = get_package_share_directory('graph_based_navigation_system')

        # check source tree (dev)
        src_path = os.path.normpath(os.path.join(pkg_dir, '..', '..', 'src', 'graph_based_navigation_system', 'config', self.GRAPH_NAME))
        if os.path.exists(src_path):
            self.get_logger().info(f'Using graph file from source tree: {src_path}')
            return src_path

        # check install space
        install_path = os.path.normpath(os.path.join(pkg_dir, 'config', self.GRAPH_NAME))
        if os.path.exists(install_path):
            self.get_logger().info(f'Using graph file from install space: {install_path}')
            return install_path

        # try package share directly
        direct_path = os.path.join(pkg_dir, 'config', self.GRAPH_NAME)
        if os.path.exists(direct_path):
            self.get_logger().info(f'Using graph file from package share: {direct_path}')
            return direct_path

        raise FileNotFoundError(f'Graph file {self.GRAPH_NAME} not found. Checked:\n - {src_path}\n - {install_path}\n - {direct_path}')

    def load_graph(self):
        with open(self.graph_file, 'r') as fh:
            data = yaml.safe_load(fh)

        # --- Load nodes ---
        raw_nodes = data.get('nodes', {})

        for nid, props in raw_nodes.items():
            nid = int(nid)
            node_data = props.copy()

            # Extract navigation config if present
            nav_cfg = node_data.pop('navigation_config', {})
            merged_nav_cfg = NavigationDefaults.NODE_DEFAULTS.copy()
            merged_nav_cfg.update(nav_cfg)

            # Flatten navigation config into node attributes
            node_data.update(merged_nav_cfg)

            # Save node attributes
            self.nodes[nid] = node_data

            # Add bare node to DG
            self.DG.add_node(nid)

        # --- Load edges ---
        raw_edges = data.get('edges', [])
        weighted_edges = []

        for e in raw_edges:
            frm = int(e['from'])
            to = int(e['to'])

            # Defaults if missing
            weight = e.get('weight', NavigationDefaults.EDGE_DEFAULTS['weight'])
            one_way = e.get('one_way', NavigationDefaults.EDGE_DEFAULTS['one_way'])

            # Merge defaults except weight & one_way
            edge_attrs = {k: v for k, v in NavigationDefaults.EDGE_DEFAULTS.items()
                        if k not in ('weight', 'one_way')}
            edge_attrs.update({k: v for k, v in e.items() if k not in ('from', 'to', 'weight', 'one_way')})

            # Save full attributes (no weight, no one_way)
            self.edges[(frm, to)] = edge_attrs

            # Always add forward edge to DG (with weight only)
            weighted_edges.append((frm, to, weight))

            # If two-way, add reverse edge as well
            if not one_way:
                #self.edges[(to, frm)] = edge_attrs
                weighted_edges.append((to, frm, weight))

        # Add weighted edges to DG
        self.DG.add_weighted_edges_from(weighted_edges)

        self.get_logger().info(
            f'Graph with {len(self.nodes)} nodes and {self.DG.number_of_edges() // 2} edges.'
        )
        '''
        # Log detailed nodes
        self.get_logger().info("Nodes:\n" + yaml.dump(self.nodes, sort_keys=False))
        
        # Format edges for logging
        edges_pretty = []
        for (frm, to), attrs in self.edges.items():
            edge_dict = {"from": frm, "to": to}
            edge_dict.update(attrs)
            edges_pretty.append(edge_dict)

        # Log pretty edges
        self.get_logger().info("Edges:\n" + yaml.dump(edges_pretty, sort_keys=False))
        '''

    def odometry_callback(self, msg):
        if self.pose_received:
            return
        
        self.pose_received = True
        
        # Extract position and orientation
        position = msg.pose.pose.position
        x = position.x
        y = position.y

        orientation = msg.pose.pose.orientation
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        roll, pitch, yaw = euler_from_quaternion(quat)

        self.mir_pose = (x, y, yaw)
        self.get_logger().info(f'Robot Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}')

        # Optional: Destroy subscription to prevent further callbacks
        self.destroy_subscription(self.robot_position_sub)        

    def get_closest_node(self):
        best = None
        best_dist = float('inf')
        for nid, props in self.nodes.items():
            dx = self.mir_pose[0] - props['x']
            dy = self.mir_pose[1] - props['y']
            d = math.hypot(dx, dy)
            if d < best_dist:
                best = nid
                best_dist = d
        return best
    
    def send_goal(self, x, y, yaw):
        goal_msg = NavigateToPose.Goal()

        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        goal_msg.pose.pose.orientation = yaw_to_quaternion(yaw)

        self.get_logger().info(f'Sending goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f} rad)')
        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected!')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: distance remaining {feedback.distance_remaining:.2f} m')

    def result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Goal reached successfully! ✅')
        else:
            self.get_logger().error(f'Failed with error code: {result.error_code}')

    def get_node_data(self, node_id: int):
        """
        Return the stored data for a given node ID from self.nodes.
        Raises KeyError if the node does not exist.
        """
        if node_id not in self.nodes:
            self.get_logger().warn(f"Node {node_id} not found in graph.")
            return None
        
        node_data = self.nodes[node_id]
        x = node_data['x']
        y = node_data['y']
        yaw = node_data['yaw']
        self.get_logger().info(f"Node {node_id} -> x={x}, y={y}, yaw={yaw}")
        return x, y, yaw, node_data
    
    def get_edge_data(self, node_a: int, node_b: int):
        """
        Return the edge attributes between node_a and node_b.
        Looks in both directions (a->b and b->a).
        Returns None if no edge exists.
        """
        # Try forward direction
        if (node_a, node_b) in self.edges:
            return self.edges[(node_a, node_b)]
        
        # Try reverse direction
        if (node_b, node_a) in self.edges:
            return self.edges[(node_b, node_a)]
        
        self.get_logger().warn(f"No edge found between {node_a} and {node_b}")
        return None

    # ----------------------
    # Navigation execution
    # ----------------------
    def execute_path(self, path):
        if len(path) < 2:
            self.get_logger().info('Path too short; nothing to do.')
            return

        for i in range(len(path) - 1):
            from_node = path[i]
            to_node = path[i + 1]

            edge_data = self.get_edge_data(from_node, to_node)
            if edge_data is None:
                self.get_logger().error(f'No edge data between {from_node} and {to_node}, cannot proceed.')
                return
            
            goal_x, goal_y, goal_yaw, to_node_data = self.get_node_data(to_node) 
            ##### MISSING THE DYNAMIC RECONFIGURATION NAV2 STUFF HERE #####     
            self.send_goal(goal_x, goal_y, goal_yaw)

        self.get_logger().info(f'Completed path to node {path[-1]}.')

    # ----------------------
    # Interactive prompt
    # ----------------------
    def prompt_user(self):
        # single-run interactive loop
        try:
            # if we stored the timer, cancel it here
            if hasattr(self, "_timer") and self._timer is not None:
                try:
                    self.destroy_timer(self._timer)
                except Exception as e:
                    self.get_logger().warn(f"Failed to cancel timer: {e}")
                self._timer = None
        except Exception as e:
            self.get_logger().warn(f"Timer cleanup failed: {e}")

        if self.mir_pose is None:
            self.get_logger().error('Could not obtain robot position. Ensure TF from map->base_link is published.')
            return

        self.get_logger().info(f'Robot closest node: {self.get_closest_node()}')
        
        start_node = None
        while start_node is None:
            try:
                user = input('Enter start node ID (or -1 to exit): ').strip()
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info('Input ended by user, exiting prompt.')
                return

            try:
                start_node = int(user)
            except ValueError:
                print('Invalid input, enter an integer node id.')
                continue

            if start_node == -1:
                self.get_logger().info('Exiting interactive navigation.')
                return

            if start_node not in self.nodes:
                print(f'Node {start_node} not present in graph. Available nodes: {list(self.nodes.keys())}')
                start_node = None
                continue
                             
        x, y, yaw, node_data = self.get_node_data(start_node)
        self.send_goal(x, y, yaw)     

        while True:
            # Get list of node IDs from the graph
            node_ids = list(self.DG.nodes())
            print(f"Available nodes: {node_ids}")
            try:
                user = input('Enter goal node ID (or -1 to exit): ').strip()
            except (EOFError, KeyboardInterrupt):
                self.get_logger().info('Input ended by user, exiting prompt.')
                return

            try:
                goal_node = int(user)
            except ValueError:
                print('Invalid input, enter an integer node id.')
                continue

            if goal_node == -1:
                self.get_logger().info('Exiting interactive navigation.')
                return

            if goal_node not in node_ids:
                print(f'Node {goal_node} not present in graph. Available nodes: {node_ids}')
                continue

            try:
                path = nx.shortest_path(self.DG, source=start_node, target=goal_node, weight='weight', method='dijkstra') # Supported options: ‘dijkstra’, ‘bellman-ford’
                total_cost = nx.shortest_path_length(self.DG, source=start_node, target=goal_node, weight='weight')
                self.get_logger().info(f"Shortest path from {start_node} to {goal_node}: {path} (cost: {total_cost})")
                self.execute_path(path)
                start_node = goal_node
            except nx.NetworkXNoPath:
                self.get_logger().info(f"No path found from {start_node} to {goal_node}")
                path = []
                total_cost = None
                continue


    # ----------------------
    # Shutdown helper
    # ----------------------
    def destroy_node(self):
        # ensure any cleanup if necessary
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GraphNavController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt received, shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
