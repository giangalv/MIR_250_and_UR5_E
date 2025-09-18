import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters


class Nav2DynamicReconfig(Node):
    """Utility node for dynamically updating Nav2 parameters."""

    def __init__(self, node_name="nav2_dynamic_reconfig"):
        super().__init__(node_name)
        self.__clients = []

    def _make_param(self, name, value):
        """Create a Parameter message with correct type."""
        if isinstance(value, bool):
            param = Parameter(name=name, type_=Parameter.Type.BOOL, value=value)
        elif isinstance(value, int):
            param = Parameter(name=name, type_=Parameter.Type.INTEGER, value=value)
        elif isinstance(value, float):
            param = Parameter(name=name, type_=Parameter.Type.DOUBLE, value=value)
        elif isinstance(value, str):
            param = Parameter(name=name, type_=Parameter.Type.STRING, value=value)
        elif isinstance(value, list):
            if all(isinstance(v, float) for v in value):
                param = Parameter(name=name, type_=Parameter.Type.DOUBLE_ARRAY, value=value)
            elif all(isinstance(v, int) for v in value):
                param = Parameter(name=name, type_=Parameter.Type.INTEGER_ARRAY, value=value)
            else:
                param = Parameter(name=name, type_=Parameter.Type.STRING_ARRAY, value=value)
        else:
            raise TypeError(f"Unsupported parameter type for {name}: {type(value)}")

        return param.to_parameter_msg()

    def _get_client(self, nav2_node: str):
        """Return cached service client for a Nav2 node."""
        if nav2_node not in self._clients:
            service_name = f'/{nav2_node}/set_parameters'
            client = self.create_client(SetParameters, service_name)
            if not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().error(f"Service not available: {service_name}")
                return None
            self._clients[nav2_node] = client
        return self._clients[nav2_node]

    def set_param(self, nav2_node: str, param_name: str, value):
        """Set a single parameter on a Nav2 node."""
        client = self._get_client(nav2_node)
        if client is None:
            return False

        request = SetParameters.Request()
        try:
            request.parameters = [self._make_param(param_name, value)]
        except TypeError as e:
            self.get_logger().error(str(e))
            return False

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result and result.results[0].successful:
            self.get_logger().info(f"{nav2_node}: {param_name} -> {value}")
            return True
        else:
            self.get_logger().warn(f"Failed to set {param_name} on {nav2_node}")
            return False

    def set_multiple(self, nav2_node: str, param_dict: dict):
        """Set multiple parameters on a Nav2 node."""
        client = self._get_client(nav2_node)
        if client is None:
            return False

        request = SetParameters.Request()
        try:
            request.parameters = [self._make_param(k, v) for k, v in param_dict.items()]
        except TypeError as e:
            self.get_logger().error(str(e))
            return False

        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result and all(r.successful for r in result.results):
            self.get_logger().info(f"{nav2_node}: updated {len(param_dict)} params")
            return True
        else:
            self.get_logger().warn(f"Failed multiple param set on {nav2_node}")
            if result:
                for r, (k, v) in zip(result.results, param_dict.items()):
                    if not r.successful:
                        self.get_logger().warn(f"  ✗ {k} = {v} ({r.reason})")
            return False
