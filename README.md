# mir_250_ros2
This is a ROS2 package for MiR 250 and UR5e series.

# Installation

## Preliminaries
## ROS2
If you haven't already installed [ROS2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) on your PC, you need to add the ROS2 apt repository.


## Source install
```bash
# create a ros2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/

# clone mir_robot into the ros2 workspace
git clone https://github.com/giangalv/mir_robot_simulation.git

# use vcs to fetch linked repos
# $ sudo apt install python3-vcstool
vcs import < src/mir_robot/ros2.repos src --recursive

# use rosdep to install all dependencies (including ROS itself)
sudo apt update
sudo apt install -y python3-rosdep

sudo apt install ros-humble-foxglove-bridge
sudo apt install ros-humble-librealsense2*

rosdep update --rosdistro=humble
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

# build all packages in the workspace
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
```

You must source the workspace in each terminal you want to work in:
```bash
source ~/ros2_ws/install/setup.bash
```

# MiR Robot Integration – Setup Instructions

> ⚠️ **IMPORTANT:**  
> Before running any scripts or commands in this repository, you must replace all placeholder values with your actual robot and authentication details.

## 🔧 Setup Configuration

In all scripts, config files, or launch files, make sure to replace:

- `<robot-ip>` with the **actual IP address** of your MiR robot.  
  Example: `http://192.168.12.34` or `http://130.251.13.90`

- `<your base64-encoded credentials>` with your **Base64-encoded `username:password`** for the MiR REST API.  
  You can find it inside the MIR dashboard Help -> API documentation -> after enter with Username & User password.

# Fix time synchronization manually
* In the Mir dashboard (mir.com in the Mir-Wifi), go to "Service" ->
  "Configuration" -> "System settings" -> "Time settings" -> "Set device time
  on robot"

Use **load from device** to sync with the system time!

### Test the connection is working

This tests the connection if returning the result from the MIR the 'MIR_IP_ADDR' and
  'MIR_API_KEY' are correct.

```bash
curl -X GET "http://<robot-ip>/api/v2.0.0/status" \
     -H "Authorization: Basic <your base64-encoded credentials>"

```

### **Fix time synchronization using ROS2:**
From the package `mir_restapi` a node called `mir_restapi_server` can be run, which can execute a time sync REST API call from the driver's host machine to the Mir's host.

* Launch the sh file with the API key and mir hostname's IP address changed

```bash
cd ~/ros2_ws/mir_robot_simulation/src

chmod +x time_synchronization_MIR250.sh

./time_synchronization_MIR250.sh
```
Change inside `time_synchronization_MIR250.sh` the:
  ros2_ws -> your workspace
  mir_hostname := 'MIR_IP_ADDR'
  mir_restapi_auth := 'YOUR_API_KEY' # for distributor

Inside the call:
    ros2 run mir_restapi mir_restapi_server --ros-args -p mir_hostname:="MIR_IP_ADDR" -p mir_restapi_auth:="YOUR_API_KEY" #distributor

* Otherwise, call them from two terminals

```bash 1
  ros2 run mir_restapi mir_restapi_server --ros-args -p mir_hostname:="MIR_IP_ADDR" -p mir_restapi_auth := "YOUR_API_KEY"
```

```bash 2
ros2 service call /mir_sync_time std_srvs/Trigger
 
```

#### **After time sync**
Keep in mind, that the time sync causes the mir_bridge to freeze. Therefore online time syncs are not recommended.


# RVIZ demo test
```bash
### rviz:
ros2 launch mir_description mir_display_launch.py
```
This should work and you are going to see the robot spawned in **rviz**.

# Dual Laser Merger
## Acknowledgement
The 3d files for MiR 250 are from [DFKI](https://github.com/DFKI-NI/mir_robot).

## Example for the Laser Merger
Before, you need to run the RVIZ demo test.
```bash 1
ros2 launch mir_description mir_display_launch.py
```
This demo shows merging of laser scan data from 2 lidars.
```bash 2
ros2 launch sensors_launcher_mir_250 demo_laser_merger.launch.py
```

## Requirements
1. Lidar 1 scan topic, the messages in the topic are required to have `frame_id`.
   ```
   ~$ ros2 topic info /f_scan
      Type: sensor_msgs/msg/LaserScan
   ```
2. Lidar 2 scan topic, the messages in the topic are required to have `frame_id`.
   ```
   ~$ ros2 topic info /b_scan
      Type: sensor_msgs/msg/LaserScan
   ```
3. TF from Lidar 1 (`front_laser_link`) and Lidar 2 (`back_laser_link`) to Target frame (`base_link`)
  ```
  ~$ ros2 topic echo /tf_static 
  transforms:
    - header:
        stamp:
          sec: 1729076136
          nanosec: 564956753
        frame_id: base_link
      child_frame_id: front_laser_link
      transform:
        translation:
          x: 0.321967
          y: 0.221817
          z: 0.0
        rotation:
          x: 0.3826834321814926
          y: 0.9238795325873352
          z: 3.9573888241688663e-14
          w: -9.553981776262265e-14
  
  transforms:
    - header:
        stamp:
          sec: 1729076136
          nanosec: 580013258
        frame_id: base_link
      child_frame_id: back_laser_link
      transform:
        translation:
          x: -0.321967
          y: -0.221817
          z: 0.0
        rotation:
          x: -0.9238795324744832
          y: 0.38268343245394154
          z: -9.553981775095244e-14
          w: -3.957388826986303e-14
  ```

# MOVE THE MIR 250
### Teleoperate the robot (optional) via the MIR network interface
* Turn the key on the robot in Manual control, go to the MIR network interface, press the *Manual control* button. The *RESUME* button, on the robot, starts to blinking in blue, press it and the led's robot changes from RED in BLUE (the robot state changes from Emergency Stop to Manual Control state). Now you can move the robot via the joystick on the MIR interface.


### Relocalize robot (optional)
If the robot's localization is lost:

* go to "Service" -> "Command view" -> "Set start position" and click + drag to current position of robot in the map
* click "Adjust"


## Start the ROS driver
```bash
ros2 launch mir_driver mir_launch.py
```

* The driver automatically launches **rviz2** to visualize the robot description, the topics and sensor
  messages.

## Start the Manual control controller via the keyboard, real joystick and whatever

* Turn the key on the robot in Autonomous control, the *RESUME* button starts to blinking in blue, press it and the led's robot changes from RED in YELLOW (the robot state changes from Emergency Stop to Pause state).

Open a new terminal and launch:
``` bash
ros2 launch mir_manual_navigation manual_control_launch.py
```
The led's robot changes from YELLOW to GREEN (the robot state changes from Pause state to Ready state).

* **Before launcing the manual_control_launch** command be sure to have the mir_launch command running.

* The driver automatically launches a seperate **teleop** window to manually move the robot using your keyboard. And add the possibility to use whatever you want adding to the twist_mux configuration the topics.

# Mapping on MiR

### Option 1: Launching the modules separately

```bash 1
### driver:
ros2 launch mir_driver mir_launch.py
```
``` bash 2
### mapping (slam_toolbox):
ros2 launch mir_navigation mapping.py 
```
``` bash 3
### navigation:
ros2 launch mir_manual_navigation manual_control_launch.py
```

### Option 2: Use combined launch file

```bash
### combined launch file:
ros2 launch mir_navigation mir_mapping_launch.py
```

On Mapping
----------
### How to map

To save the created map, use the rviz plugin **"Save Map"** and **"Serialize
Map"**. From time to time, segmentation faults or timeouts occur, so make sure
your map is saved before shutting down the connection.


### Refine existing map

The given launch commands will create a fresh new map of the environment. If
you like to adapt an existing map (must be serialized!) you can **deserialize
it** using the rviz slam_toolbox plugin.


# Navigation on MiR

### Option 1: Launching the modules separately

```bash
### driver:
ros2 launch mir_driver mir_launch.py

### localization (amcl)
ros2 launch mir_navigation amcl.py use_sim_time:=false map:={path to existing map}

### navigation
ros2 launch mir_navigation navigation.py use_sim_time:=false
```

### Option 2: Use combined launch file

```bash
### combined launch file:
ros2 launch mir_navigation mir_nav_launch.py map:={path to /name of existing map or *} 
```
* just the name .yaml  inside the maps folder inside mir_navigation like `ros2 launch mir_navigation mir_nav_launch.py map:=name_map.yaml`
it launchs the driver and localization (amcl) using an existing map.`

### Workflow

Once the driver is running and rviz is started, you need to **set
the initial pose** on the map. This doesn't have to be accurate, just a
reference, and amcl will do the refinement. To refine, **move the robot** around
a little using the teleop window and the scan will eventually match the map.


# Graph-Based Navigation System for the MIR 

## Graph Creation Tutorial

After creating the base map using SLAM or mapping tools on the MiR (resulting in a .pgm or .png image and a .yaml metadata file), there is an intermediate step before actual robot navigation:

### Step 1: Prepare the Map Files
Locate the generated map files (typically in **~/mir_navigation/maps** )

```text
my_map.pgm
my_map.yaml
```

Copy them to the graph system's map directory:
```bash
mkdir -p ~/src/graph_based_navigation_system/maps/
cp ~/src/mir_navigation/maps/my_map.* ~/src/graph_based_navigation_system/maps/
```

### Step 2: Launch the Graph Editor
```bash
ros2 run graph_based_navigation_system graph_editor 
```

You'll see:
- The map dispayed in a new window
- control instructions in the terminal

### Step 3: Create Navigation Nodes
1. **Add Nodes** (Left-click)
2. **Set Orientations** (Shift+Drag):
  - Click and hold a node
  - While holdind Shift, drag to set the robot's orientation
  - Release to confirm

### Step 4: Connect Nodes with Edges
1. **Create Edges** (Right-click)
  - Right-click on first node 
  - Right-click on second node

### Step 5: Save the Grap
Simply close the window to automatically save.

### Step 6: Generate Visual Map
```bash
ros2 run graph_based_navigation_system  render_graph_on_map
```
This creates:
- `map_graph.yaml` - A visual representation of the map, enhanced with nodes, edges, and orientations overlaid on the original map image.
- `map_with_nodes_edges.png` - An annotated version of the original map, useful for reference during development and visualization in RViz for navigation tasks.
- `map_with_nodes_edges.yaml` - A copy of the original map metadata file, updated to reference the new annotated image.

Before starting navigation, you should customize the`map_graph.yaml`file to adjust node positions and connections according to your specific navigation needs and environment layout. These modifications can be done either before generating the final map or afterward if adjustments are needed.

Once the files are ready, you need to move them to the appropriate locations in the `graph_based_navigation_system` package:
* Place `map_graph.yaml` in the `/config ` folder.
* Place both  `map_with_nodes_edges.png` and `map_with_nodes_edges.yaml` in the `/maps` folder.

Make sure all files are in their correct locations before beginning navigation operations. This ensures the system can properly access all required map data and configuration files.

## MIR Navigation + Graph-Based Navigation
This package provides launch files for running the **MIR navigation stack** (AMCL + Nav2 + manual navigation + MIR driver) along with a **graph-based navigation controller**.
The system is split into two launch files for flexibility:

- `grap_navigation_launch.py` - Launches the MIR driver, AMCL localization and Nav2 stack.
- `graph_nav_controller_launch.py` - Launches the Graph Navigation Controller node separately.

### Launch Files Overview

1. **graph_navigation_launch.py**
This file starts the main navigation system, including:

* **MIR Driver** (`mir_driver` package)
* **Manual Navigation Tools** (`mir_manual_navigation` package)
* **AMCL Localization** (`amcl.py`)
* **Nav2 Stack** (`navigation.py`)

**Usage:**
```bash
ros2 launch graph_based_navigation_system graph_navigation_launch.py map:=path/to/map.yaml
```

**Arguments:**
* `map` - Path to the map `.yaml` file (required).
* `use_sim_time` - Set to `true` for simulation (defaul:`false`)

2. **graph_nav_controller_launch.py**
This file starts **only** the `graph_nav_controller` node, which controls navigation based on a graph of waypoints.

**Usage:**
```bash
ros2 launch graph_based_navigation_system graph_nav_controller_launch.py
```

**Arguments:**
* `use_sim_time` - Set to `true` for simulation (defaul:`false`)


#########################################################################
# Gazebo demo (mapping)

```bash
### gazebo:
ros2 launch mir_gazebo mir_gazebo_launch.py  world:=maze

### mapping (slam_toolbox)
ros2 launch mir_navigation mapping.py use_sim_time:=true slam_params_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/config/mir_mapping_async_sim.yaml

### navigation (optional)
ros2 launch mir_navigation navigation.py use_sim_time:=true cmd_vel_w_prefix:=/diff_cont/cmd_vel_unstamped
```

# Gazebo demo (Navigation with existing map)

```bash
### gazebo
ros2 launch mir_gazebo mir_gazebo_launch.py world:=maze rviz_config_file:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/rviz/mir_nav.rviz


### localization (existing map)
ros2 launch mir_navigation amcl.py use_sim_time:=true map:=$(ros2 pkg prefix mir_navigation)/share/mir_navigation/maps/maze.yaml

### navigation
ros2 launch mir_navigation navigation.py use_sim_time:=true
```
##########################################################################



