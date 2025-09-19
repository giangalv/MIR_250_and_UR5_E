#!/usr/bin/env python3
import os
import math
import yaml
import networkx as nx
import sys
import rclpy
import time

from geometry_msgs.msg import Pose2D
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import PoseStamped
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory


# NOTE: This import assumes you have these helper classes available as in your repo.
# - NavigationDefaults: holds defaults for nodes/edges and style profiles
# - Nav2DynamicReconfig: small wrapper to set params on Nav2 nodes
from graph_based_navigation_system.navigation_helper import BasicNavigator
from graph_based_navigation_system.navigation_defaults import NavigationDefaults
from graph_based_navigation_system.nav2_dynamic_reconfig import Nav2DynamicReconfig

_GRAPH_NAME = 'test_TER_1'  # Default graph name, to adjust as needed

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
        # Nav2 dynamic reconfig utility
        self.nav2_reconfig = Nav2DynamicReconfig()

        # Basic navigator helper
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        # graph structures
        self.nodes = {}
        self.edges = {}
        self.DG = nx.DiGraph()

        # load graph
        self.graph_file = self.get_graph_path()
        self.load_graph()

        self.get_logger().info('GraphNavController initialized and ready.')

        # File to save/load the last position
        self.file_path = os.path.join(os.path.expanduser("~"), ".ros", "mir_position.txt")
        self.mir_pose = self.load_position()

        # Timer to handle prompt
        self.create_timer(0.5, self.prompt_user)

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
        ### PRINT GRAPH FOR DEBUGGING ###
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
    
    def load_position(self):
        """Load position from file once."""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, "r") as f:
                    line = f.readline().strip()
                    if line:
                        x, y, theta = map(float, line.split())
                        self.get_logger().info(f"Mir position loaded: x={x}, y={y}, theta={theta}")
                        return Pose2D(x=x, y=y, theta=theta)
            except Exception as e:
                self.get_logger().warn(f"Failed to load position: {e}")
        else:
            self.get_logger().info("No previous position found, starting from (0,0,0)")
            return Pose2D(x=0.0, y=0.0, theta=0.0)
        return None 

    def get_closest_node(self):
        best = None
        best_dist = float('inf')

        if self.mir_pose is None:
            self.get_logger().warn("Robot pose not available yet.")
            return None

        for nid, props in self.nodes.items():
            dx = self.mir_pose.x - props['x']
            dy = self.mir_pose.y - props['y']
            d = math.hypot(dx, dy)
            if d < best_dist:
                best = nid
                best_dist = d

        return best
    
    def get_node_data(self, node_id: int):
        """
        Return the stored data for a given node ID from self.nodes.
        Ensures a navigation_config dict is always present.
        """
        if node_id not in self.nodes:
            self.get_logger().warn(f"Node {node_id} not found in graph.")
            return None

        node_data = dict(self.nodes[node_id])  # make a copy
        node_data.setdefault("navigation_config", {})  # ensure key exists

        x = node_data['x']
        y = node_data['y']
        yaw = node_data['yaw']
        #self.get_logger().info(f"Node {node_id} -> x={x}, y={y}, yaw={yaw}")
        return x, y, yaw, node_data

    def get_edge_data(self, node_a: int, node_b: int):
        """
        Return the edge attributes between node_a and node_b.
        Looks in both directions (a->b and b->a).
        Ensures a navigation_config dict is always present.
        """
        edge = None

        # Try forward direction
        if (node_a, node_b) in self.edges:
            edge = dict(self.edges[(node_a, node_b)])
        # Try reverse direction
        elif (node_b, node_a) in self.edges:
            edge = dict(self.edges[(node_b, node_a)])

        if edge is None:
            self.get_logger().warn(f"No edge found between {node_a} and {node_b}")
            return None

        edge.setdefault("navigation_config", {})  # ensure key exists
        return edge

    # ----------------------
    # Navigation execution
    # ----------------------
    def execute_path(self, path):
        if len(path) < 2:
            self.get_logger().info('You are already at the goal node or path is too short.')
            return

        for i in range(len(path) - 1):
            from_node = path[i]
            to_node = path[i + 1]

            # --- Get edge parameters ---
            edge_data = self.get_edge_data(from_node, to_node)
            if edge_data is None:
                self.get_logger().error(f"No edge data between {from_node} and {to_node}, cannot proceed.")
                return

            # --- Get node parameters ---
            goal_x, goal_y, goal_yaw, node_data = self.get_node_data(to_node)

            # --- Set navigation defaults parameters ---

            # --- Apply dynamic reconfig (from node_data and the edge_data) for the edge

            # --- Execute navigation ---
            self.get_logger().info(f"➡️ Sending robot to node {to_node}: ({goal_x:.2f}, {goal_y:.2f}, yaw={goal_yaw:.2f})")
            success = self.go_to_pose_and_wait(goal_x, goal_y, goal_yaw)
            if not success:
                self.get_logger().error(f"❌ Failed to reach node {to_node}, aborting path.")
                return

            self.get_logger().info(f"✅ Arrived at node {to_node}, waiting before next...")
            time.sleep(5)  # configurable pause ### future labels actions

        self.get_logger().info(f"🎉 Completed path to node {path[-1]}.")

    # ----------------------
    # Nav2 execution wrapper
    # ----------------------
    def go_to_pose_and_wait(self, x, y, yaw):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation = yaw_to_quaternion(yaw)

        self.get_logger().info(f"📍 Sending goal: ({x:.2f}, {y:.2f}, yaw={yaw:.2f})")
        self.navigator.goToPose(goal)

        while not self.navigator.isNavComplete():
            feedback = self.navigator.getFeedback()
            if feedback:
                self.get_logger().info(
                    f"⏳ Distance remaining: {feedback.distance_remaining:.2f} m, "
                )
            rclpy.spin_once(self.navigator, timeout_sec=0.1)

        result = self.navigator.getResult()
        if result == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("🎯 Goal reached (distance ≤ tolerance)")
            return True
        elif result == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn("⚠️ Goal canceled")
        elif result == GoalStatus.STATUS_ABORTED:
            self.get_logger().error("❌ Goal aborted")
        else:
            self.get_logger().warn(f"⚠️ Goal finished with unknown status {result}")
        return False
    
    # ----------------------
    # Interactive prompt
    # ----------------------
    def prompt_user(self):
        if self.mir_pose is None:
            self.get_logger().info('Waiting for AMCL pose...')
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
                raise SystemExit

            if start_node not in self.nodes:
                print(f'Node {start_node} not present in graph. Available nodes: {list(self.nodes.keys())}')
                start_node = None
                continue
                             
        goal_x, goal_y, goal_yaw, node_data = self.get_node_data(start_node)
        # --- Execute navigation ---
        success = self.go_to_pose_and_wait(goal_x, goal_y, goal_yaw)
        if not success:
            self.get_logger().error(f"❌ Failed to reach node {start_node}, aborting path.")
            return 

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
                raise SystemExit

            if goal_node not in node_ids:
                print(f'Node {goal_node} not present in graph. Available nodes: {node_ids}')
                continue
            else:
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
    except SystemExit:
        pass  # clean exit from goal_node == -1
    finally:
        node.destroy_node()
        rclpy.shutdown()



if __name__ == '__main__':
    main()
