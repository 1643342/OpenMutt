cmake_minimum_required(VERSION 3.8)
project(champ_msgs)

# Required ROS 2 dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate message interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Velocities.msg"
  "msg/PID.msg"
  "msg/Imu.msg"
  "msg/Point.msg"
  "msg/PointArray.msg"
  "msg/Joints.msg"
  "msg/Contacts.msg"
  "msg/ContactsStamped.msg"
  "msg/Pose.msg"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

# Export package dependencies
ament_export_dependencies(rosidl_default_runtime std_msgs geometry_msgs builtin_interfaces)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
