class NavigationDefaults:
    """Centralized default values for navigation graph nodes and edges"""
    
    # Node defaults
    NODE_DEFAULTS = {
        'xy_tolerance': 0.1,      # xy_goal_tolerance: 0.25 #Positional tolerance to goal (meters)
        'yaw_tolerance': 0.05     # yaw_goal_tolerance: 0.05 #Angular tolerance to goal (radians)
    }

    # Edge defaults
    EDGE_DEFAULTS = {
        'weight': 1.0,              # Default edge weight
        'one_way': False,           # Default edge direction
        'max_vel_x': 0.26,           # Max linear velocity (m/s)
        'max_vel_theta': 1.0,       # Max angular velocity (rad/s)
        'acc_lim_x': 2.5,           # Linear acceleration limit (m/s^2)
        'acc_lim_theta': 3.2,       # Angular acceleration limit (rad/s^2)
        'planner': "SmacPlanner",   # Nav2 planner plugin name
        'controller': "DWB",        # Controller plugin name
        'inflation_radius': 0.5,    # Costmap inflation radius (meters)
        'cost_scaling_factor': 3.0, # Costmap scaling factor
        'obstacle_max_range': 3.0,  # Obstacle detection range (meters)
        'obstacle_min_range': 0.05, # Minimum obstacle detection range (meters)
        #'max_obstacle_height': 3.0, # Max height of obstacles (meters
        'raytrace_max_range': 3.5,  # Max range for ray tracing (meters)
        'raytrace_min_range': 0.05  # Minimum range for ray tracing (meters)
    }

    @classmethod
    def get_default_config(cls):
        """Returns the complete default configuration"""
        return {
            'node': cls.NODE_DEFAULTS,
            'edge': cls.EDGE_DEFAULTS
        }