#!/usr/bin/env python3
import os
import math
import time
import yaml
import networkx as nx

import rclpy
from rclpy.node import Node
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


def euclidean_distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


class GraphNavController(Node):
    def __init__(self,):
        super().__init__('graph_nav_controller')
        
        _GRAPH_NAME = 'test_TER_1'  # Default graph name, can be overridden
        # config
        self.GRAPH_NAME = f'{_GRAPH_NAME}_graph.yaml'
        self.defaults = NavigationDefaults()

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # action client
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # internal state
        self.nodes = {}
        self.G = nx.Graph()

        # load graph
        self.graph_file = self.get_graph_path()
        self.load_graph()

        self.get_logger().info('GraphNavController initialized and ready.')

        # small delay to let TF and nav2 come up
        self.get_logger().info('Waiting for navigate_to_pose action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('NavigateToPose server not available yet. Will wait when sending goals.')

        # start the interactive prompt after short delay to ensure node finishes init
        # use a timer so rclpy spin can run concurrently
        self.create_timer(1.0, self._start_prompt_once)
        self._prompt_started = False

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

        # Load nodes as a dict {id: attributes}
        raw_nodes = data.get('nodes', {})
        self.nodes = {int(k): v for k, v in raw_nodes.items()}

        # Build graph with raw YAML data (no defaults yet)
        self.G = nx.Graph()
        for nid, props in self.nodes.items():
            self.G.add_node(nid, **props)

        # Load edges directly from YAML
        for e in data.get('edges', []):
            frm = e['from']
            to = e['to']
            # No defaults applied here
            edge_attrs = {k: v for k, v in e.items() if k not in ('from', 'to')}
            self.G.add_edge(frm, to, **edge_attrs)

        self.get_logger().info(
            f'Loaded graph "{self.graph_file}" with {len(self.nodes)} nodes and {self.G.number_of_edges()} edges.'
        )


    def apply_node_and_edge_params(self, from_node, to_node):
        """
        Apply navigation parameters for a specific segment (edge) from `from_node` to `to_node`.
        Combines node tolerances and edge-specific velocity/planner/controller parameters.
        """
        if not self.G.has_edge(from_node, to_node):
            self.get_logger().warn(f"No edge from {from_node} to {to_node}")
            return False

        edge_data = self.G.edges[from_node, to_node].get('navigation_config', {})
        from_node_data = self.G.nodes[from_node].get('navigation_config', {})
        to_node_data = self.G.nodes[to_node].get('navigation_config', {})

        # Combine node tolerances (take min or max as needed)
        combined_tolerances = {
            'xy_tolerance': min(from_node_data.get('xy_tolerance', 0.25),
                                to_node_data.get('xy_tolerance', 0.25)),
            'yaw_tolerance': min(from_node_data.get('yaw_tolerance', 0.05),
                                to_node_data.get('yaw_tolerance', 0.05)),
        }

        # Merge edge navigation parameters
        nav_params = {**edge_data, **combined_tolerances}

        # Apply dynamically to Nav2 nodes
        nav2_nodes = ['controller_server', 'planner_server', 'bt_navigator']
        success = True
        for node_name in nav2_nodes:
            if not Nav2DynamicReconfig.set_multiple(node_name, nav_params):
                success = False

        # Log applied parameters
        self.get_logger().info(f"Applied parameters for segment {from_node} -> {to_node}: {nav_params}")
        return success


    # ----------------------
    # TF helpers
    # ----------------------
    def get_robot_position_xy(self, timeout_sec: float = 1.0):
        """Return (x, y) of base_link in map frame or (None, None) on failure."""
        try:
            now = self.get_clock().now()
            # lookup_transform will accept rclpy Time
            t = self.tf_buffer.lookup_transform('map', 'base_link', Time(seconds=0))
            x = t.transform.translation.x
            y = t.transform.translation.y
            return x, y
        except Exception as e:
            self.get_logger().warn(f'Failed to get robot position via TF: {e}')
            return None, None

    def get_closest_node(self, current_x, current_y):
        best = None
        best_dist = float('inf')
        for nid, props in self.nodes.items():
            dx = current_x - props['x']
            dy = current_y - props['y']
            d = math.hypot(dx, dy)
            if d < best_dist:
                best = nid
                best_dist = d
        return best

    # ----------------------
    # Actions / hooks
    # ----------------------
    def run_node_actions(self, node_id: int):
        """Execute actions listed in node metadata. Extend this to call services, launch other nodes, etc."""
        props = self.nodes.get(node_id, {})
        actions = props.get('actions', [])
        if not actions:
            return

        for action in actions:
            # Simple built-in action examples:
            if action == 'pause':
                self.get_logger().info(f'Action pause at node {node_id} (no duration specified).')
            else:
                # extend here to support actions like 'scan_area', 'take_picture', call services, etc.
                self.get_logger().info(f'Action "{action}" requested at node {node_id} (no handler implemented).')

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
            to_props = self.nodes[to_node]

            # 1) Apply node+edge params for this segment
            self.apply_node_and_edge_params(from_node, to_node)

            # 2) Optional pause before starting (node-based)
            #pause_before = to_props.get('pause_before', 0.0) or to_props.get('pause_duration', 0.0)
            #if pause_before and pause_before > 0.0:
            #    self.get_logger().info(f'Pausing {pause_before}s before moving to node {to_node}')
            #    time.sleep(float(pause_before))

            # 3) Build and send NavigateToPose goal
            goal = PoseStamped()
            goal.header.frame_id = 'map'
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.pose.position.x = float(to_props['x'])
            goal.pose.position.y = float(to_props['y'])
            yaw = float(to_props.get('yaw', 0.0))
            goal.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.orientation.w = math.cos(yaw / 2.0)

            nav_goal = NavigateToPose.Goal()
            nav_goal.pose = goal

            self.get_logger().info(f'Navigating to node {to_node}: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}), style={to_props.get("style", "default")}')

            # Ensure server available
            if not self.nav_client.wait_for_server(timeout_sec=10.0):
                self.get_logger().error('NavigateToPose action server not available; aborting path.')
                return

            send_goal_future = self.nav_client.send_goal_async(nav_goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            goal_handle = send_goal_future.result()
            if not goal_handle.accepted:
                self.get_logger().error(f'Goal to node {to_node} rejected by server.')
                return

            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()
            if result is None:
                self.get_logger().error(f'No result returned for goal to node {to_node}.')
                return

            if getattr(result.result, 'error_code', 1) == 0:
                self.get_logger().info(f'Reached node {to_node} successfully.')
                # run any node actions (post-arrival)
                self.run_node_actions(to_node)

                # optional pause after arrival
                pause_after = to_props.get('pause_after', 0.0)
                if pause_after and pause_after > 0.0:
                    self.get_logger().info(f'Pausing {pause_after}s after arrival at node {to_node}')
                    time.sleep(float(pause_after))
            else:
                self.get_logger().warn(f'Failed to reach node {to_node}. Error code: {result.result.error_code}')
                # decide: break or continue. Here we abort the path.
                return

        self.get_logger().info(f'Completed path to node {path[-1]}.')

    # ----------------------
    # Interactive prompt
    # ----------------------
    def _start_prompt_once(self):
        if self._prompt_started:
            return
        self._prompt_started = True
        # run prompt in a separate thread to avoid blocking timers / spins? Using input() will block,
        # but rclpy.spin still runs in the main thread. If you want non-blocking UI, change to a ROS service.
        self.timer = self.create_timer(0.1, self.prompt_user, callback_group=None)

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

        # ask TF for position
        x, y = self.get_robot_position_xy() 
        if x is None:
            self.get_logger().error('Could not obtain robot position. Ensure TF from map->base_link is published.')
            return

        start_node = self.get_closest_node(x, y)
        self.get_logger().info(f'Robot closest node: {start_node}')

        while True:
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

            if goal_node not in self.nodes:
                print(f'Node {goal_node} not present in graph. Available nodes: {list(self.nodes.keys())}')
                continue

            try:
                path = nx.astar_path(
                            self.G, start_node, goal_node,
                            heuristic=lambda u, v: euclidean_distance(self.nodes[u]['x'], self.nodes[u]['y'], self.nodes[v]['x'], self.nodes[v]['y']),
                            weight='weight'
                        )
            except nx.NetworkXNoPath:
                print(f'No path between {start_node} and {goal_node}.')
                continue

            print(f'Planned path: {path}')
            self.execute_path(path)
            start_node = goal_node

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
