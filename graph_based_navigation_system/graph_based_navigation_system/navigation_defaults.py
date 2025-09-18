class NavigationDefaults:
    """
    Centralized default values for navigation graph nodes and edges,
    plus mapping to actual Nav2 parameter names.
    """

    # ----------------------------------------------------------
    # Node defaults: Used when the robot reaches or waits at a node
    # ----------------------------------------------------------
    NODE_DEFAULTS = {
        'xy_tolerance': 0.1,       # Positional tolerance to goal (m)
        'yaw_tolerance': 0.05      # Angular tolerance to goal (rad)
    }

    # ----------------------------------------------------------
    # Edge defaults: Used while traveling along an edge between nodes
    # ----------------------------------------------------------
    EDGE_DEFAULTS = {
        'weight': 1.0,               # Pathfinding edge weight
        'one_way': False,            # Edge is bidirectional by default

        # Controller constraints
        'max_vel_x': 0.26,           # Max linear velocity (m/s)
        'max_vel_theta': 1.0,        # Max angular velocity (rad/s)
        'acc_lim_x': 2.5,            # Linear acceleration limit (m/s^2)
        'acc_lim_theta': 3.2,        # Angular acceleration limit (rad/s^2)

        # Plugins
        'planner': "SmacPlanner",    # Default planner plugin
        'controller': "DWB",         # Default controller plugin

        # Costmap parameters
        'inflation_radius': 0.5,     # Obstacle inflation radius (m)
        'cost_scaling_factor': 3.0,  # Cost steepness near obstacles
        'obstacle_max_range': 3.0,   # Max obstacle detection range (m)
        'obstacle_min_range': 0.05,  # Min obstacle detection range (m)

        # Raytracing parameters
        'raytrace_max_range': 3.5,   # Max range for clearing free space (m)
        'raytrace_min_range': 0.05   # Min range for clearing free space (m)
    }

    # ----------------------------------------------------------
    # Mapping: Graph config keys → Actual Nav2 parameter names
    # ----------------------------------------------------------
    PARAM_MAPPING = {
        # Controller tolerances
        'xy_tolerance': ("controller_server", "FollowPath.xy_goal_tolerance"),
        'yaw_tolerance': ("controller_server", "FollowPath.yaw_goal_tolerance"),

        # Controller motion limits
        'max_vel_x': ("controller_server", "FollowPath.max_vel_x"),
        'max_vel_theta': ("controller_server", "FollowPath.max_vel_theta"),
        'acc_lim_x': ("controller_server", "FollowPath.acc_lim_x"),
        'acc_lim_theta': ("controller_server", "FollowPath.acc_lim_theta"),

        # Costmap tuning (local + global can reuse the same values)
        'inflation_radius': [
            ("local_costmap", "local_costmap.local_costmap.inflation_layer.inflation_radius"),
            ("global_costmap", "global_costmap.global_costmap.inflation_layer.inflation_radius")
        ],
        'cost_scaling_factor': [
            ("local_costmap", "local_costmap.local_costmap.inflation_layer.cost_scaling_factor"),
            ("global_costmap", "global_costmap.global_costmap.inflation_layer.cost_scaling_factor")
        ],
        'obstacle_max_range': [
            ("local_costmap", "local_costmap.local_costmap.obstacle_layer.obstacle_max_range"),
            ("global_costmap", "global_costmap.global_costmap.obstacle_layer.obstacle_max_range")
        ],
        'obstacle_min_range': [
            ("local_costmap", "local_costmap.local_costmap.obstacle_layer.obstacle_min_range"),
            ("global_costmap", "global_costmap.global_costmap.obstacle_layer.obstacle_min_range")
        ],
        'raytrace_max_range': [
            ("local_costmap", "local_costmap.local_costmap.obstacle_layer.raytrace_max_range"),
            ("global_costmap", "global_costmap.global_costmap.obstacle_layer.raytrace_max_range")
        ],
        'raytrace_min_range': [
            ("local_costmap", "local_costmap.local_costmap.obstacle_layer.raytrace_min_range"),
            ("global_costmap", "global_costmap.global_costmap.obstacle_layer.raytrace_min_range")
        ],
    }

    @classmethod
    def get_default_config(cls):
        """
        Returns the complete default configuration dictionary.
        Example:
            {
                'node': {...},  # Default node parameters
                'edge': {...}   # Default edge parameters
            }
        """
        return {
            'node': cls.NODE_DEFAULTS,
            'edge': cls.EDGE_DEFAULTS
        }

    @classmethod
    def translate(cls, config: dict) -> dict:
        """
        Convert a simplified graph config dict into full Nav2 parameters.

        Example input:
            {'xy_tolerance': 0.1, 'max_vel_x': 0.2}

        Example output:
            {
                'controller_server': {
                    'FollowPath.xy_goal_tolerance': 0.1,
                    'FollowPath.max_vel_x': 0.2
                }
            }
        """
        translated = {}

        for key, value in config.items():
            if key not in cls.PARAM_MAPPING:
                continue
            mapping = cls.PARAM_MAPPING[key]

            if isinstance(mapping, list):
                # Key applies to multiple servers (e.g., local & global costmap)
                for (node, param_name) in mapping:
                    translated.setdefault(node, {})[param_name] = value
            else:
                node, param_name = mapping
                translated.setdefault(node, {})[param_name] = value

        return translated
