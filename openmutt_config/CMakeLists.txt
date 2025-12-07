cmake_minimum_required(VERSION 3.8)
project(openmutt_config)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(DIRECTORY
<<<<<<< HEAD
  launch
  config/joints
  config/gait
  config/links
=======
  launch config include maps worlds
>>>>>>> 526e07a (new file:   .vendored-hashes.txt)
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()