cmake_minimum_required(VERSION 3.5)
project(champ_description)

find_package(ament_cmake REQUIRED)

# Install URDF and XACRO files
install(DIRECTORY urdf DESTINATION share/${PROJECT_NAME})
install(DIRECTORY meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# Custom command to generate the .dsv file
set(GZ_SIM_RESOURCE_PATH_DSV "${CMAKE_CURRENT_BINARY_DIR}/share/${PROJECT_NAME}/environment/gz_sim_resource_path.dsv")
file(WRITE ${GZ_SIM_RESOURCE_PATH_DSV} "prepend-non-duplicate;GZ_SIM_RESOURCE_PATH;\$AMENT_CURRENT_PREFIX/share/champ_description\n")

install(
  FILES ${GZ_SIM_RESOURCE_PATH_DSV}
  DESTINATION share/${PROJECT_NAME}/environment
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
