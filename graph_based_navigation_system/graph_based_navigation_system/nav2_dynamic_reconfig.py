import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters

class Nav2DynamicReconfig(Node):
    def __init__(self, node_name="nav2_dynamic_reconfig"):
        super().__init__(node_name)

    def set_param(self, nav2_node, param_name, value):
        """Set a single Nav2 parameter dynamically."""
        client = self.create_client(SetParameters, f'/{nav2_node}/set_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"[Nav2DynamicReconfig] Service not available for {nav2_node}")
            return False
        
        request = SetParameters.Request()
        request.parameters = [Parameter(name=param_name, value=value).to_parameter_msg()]
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"[Nav2DynamicReconfig] {nav2_node}: {param_name} -> {value}")
            return True
        else:
            self.get_logger().warn(f"[Nav2DynamicReconfig] Failed to set {param_name} on {nav2_node}")
            return False

    def set_multiple(self, nav2_node, param_dict):
        """Set multiple parameters at once."""
        client = self.create_client(SetParameters, f'/{nav2_node}/set_parameters')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(f"[Nav2DynamicReconfig] Service not available for {nav2_node}")
            return False

        request = SetParameters.Request()
        for k, v in param_dict.items():
            request.parameters.append(Parameter(name=k, value=v).to_parameter_msg())

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f"[Nav2DynamicReconfig] Updated {len(param_dict)} params on {nav2_node}")
            return True
        else:
            self.get_logger().warn(f"[Nav2DynamicReconfig] Failed multiple param set on {nav2_node}")
            return False
