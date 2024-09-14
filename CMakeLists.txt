cmake_minimum_required(VERSION 3.5)
project(simple_planner)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories (if you have headers in a separate directory)
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
)

# Add executables
add_executable(simple_planner_node src/simple_planner_node.cpp)
add_executable(grid_map_publisher src/grid_map_publisher.cpp)
add_executable(robot_controller_node src/robot_controller_node.cpp)


# Link against dependencies
ament_target_dependencies(simple_planner_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  grid_map_msgs
  grid_map_core
)

ament_target_dependencies(grid_map_publisher
  rclcpp
  tf2_ros
  nav_msgs
  OpenCV  # Added OpenCV here for clarity
)

ament_target_dependencies(robot_controller_node
  rclcpp
  nav_msgs
  geometry_msgs
  nav2_msgs
  tf2_geometry_msgs
)






# Link libraries explicitly for grid_map_publisher
target_link_libraries(grid_map_publisher ${OpenCV_LIBS})


# Install targets
install(TARGETS
  simple_planner_node
  grid_map_publisher
  robot_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install other resources, e.g., URDF files and launch files


install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Export the package
ament_package()

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)