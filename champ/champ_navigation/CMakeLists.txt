cmake_minimum_required(VERSION 3.8)
project(champ_navigation)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_bringup REQUIRED)
find_package(slam_toolbox REQUIRED)

# Install launch and RViz files
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp tf2_ros nav2_bringup slam_toolbox)
ament_package()
