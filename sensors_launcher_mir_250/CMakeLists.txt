cmake_minimum_required(VERSION 3.8)
project(sensors_launcher_mir_250)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Handle PCL_ROOT policy warning
cmake_policy(SET CMP0074 NEW)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# Explicitly set the PCL_DIR if necessary
# set(PCL_DIR "/path/to/pcl/share/pcl/cmake")  # Uncomment and update if needed

# Modern PCL finding
find_package(PCL REQUIRED COMPONENTS common io filters)

# If PCL is found, you can print out the locations for debugging
if(PCL_FOUND)
  message(STATUS "Found PCL: ${PCL_INCLUDE_DIRS}")
else()
  message(FATAL_ERROR "PCL not found!")
endif()

# Include directories
ament_auto_add_library(sensors_launcher_mir_250 SHARED
  src/dual_laser_merger.cpp
)

target_include_directories(sensors_launcher_mir_250 PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${PCL_INCLUDE_DIRS}  # Include PCL headers
)

# Link PCL components explicitly
target_link_libraries(sensors_launcher_mir_250
  ${PCL_LIBRARIES}  # Link PCL libraries
)

# Add dependencies
ament_target_dependencies(sensors_launcher_mir_250
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  rclcpp_components
  pcl_conversions
)

# Register component
rclcpp_components_register_node(sensors_launcher_mir_250
  PLUGIN "merger_node::MergerNode"
  EXECUTABLE dual_laser_merger_node
)

# Add camera_left_changer node
add_executable(camera_left_changer_node src/camera_left_changer.cpp)
ament_target_dependencies(camera_left_changer_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Add camera_right_changer node
add_executable(camera_right_changer_node src/camera_right_changer.cpp)
ament_target_dependencies(camera_right_changer_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)

# Install targets
install(TARGETS
  sensors_launcher_mir_250
  camera_left_changer_node
  camera_right_changer_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Export dependencies
ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
  rclcpp_components
  PCL
  pcl_conversions
)
ament_export_include_directories(include)
ament_export_libraries(sensors_launcher_mir_250)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
