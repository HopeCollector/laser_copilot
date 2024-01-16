cmake_minimum_required(VERSION 3.8)
project(laser_copilot_applications)

option(OPT_CONTROLLER_USE_PX4_MSG "Use px4_msgs for controller" OFF)
option(OPT_CONTROLLER_USE_MAVROS_MSG "Use mavros_msgs for controller" ON)

add_compile_options( -std=c++2a -O3)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)

# find dependencies
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

ament_auto_add_library(obj_dist_component SHARED src/obj_dist_node.cpp)
target_link_libraries(obj_dist_component ${PCL_LIBRARIES})
rclcpp_components_register_nodes(obj_dist_component 
  "laser_copilot_applications::obj_dist"
)

ament_auto_add_library(safe_fly_controller_component SHARED src/safe_fly_controller.cc)
target_link_libraries(safe_fly_controller_component Eigen3::Eigen)
target_compile_definitions(safe_fly_controller_component
  PRIVATE "OPT_CONTROLLER_USE_PX4_MSG=$<BOOL:${OPT_CONTROLLER_USE_PX4_MSG}>"
  PRIVATE "OPT_CONTROLLER_USE_MAVROS_MSG=$<BOOL:${OPT_CONTROLLER_USE_MAVROS_MSG}>"
)
rclcpp_components_register_nodes(safe_fly_controller_component
  "laser_copilot_applications::safe_fly_controller"
)

ament_auto_add_library(traj_replayer_component SHARED src/traj_replayer.cc)
target_link_libraries(traj_replayer_component Eigen3::Eigen)
rclcpp_components_register_nodes(traj_replayer_component
  "laser_copilot_applications::traj_replayer"
)

ament_auto_add_library(visualization_helper_component SHARED src/visualization_helper.cc)
rclcpp_components_register_nodes(visualization_helper_component
  "laser_copilot_applications::visualization_helper"
)

ament_auto_add_library(sbus_bridge_component SHARED src/sbus_bridge.cc)
target_include_directories(sbus_bridge_component
  PRIVATE ${CMAKE_INSTALL_PREFIX}/../periphery/include)
target_link_directories(sbus_bridge_component BEFORE
  PRIVATE ${CMAKE_INSTALL_PREFIX}/../periphery/lib)
target_link_libraries(sbus_bridge_component periphery)
rclcpp_components_register_nodes(sbus_bridge_component
  "laser_copilot_applications::sbus_bridge"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS obj_dist_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install (
  DIRECTORY   launch
  DESTINATION share/${PROJECT_NAME}
)

# install python module for others to use
# currently donnot need this
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

# install scripts to lib/${PROJECT_NAME} is the most important
find_package(rclpy REQUIRED)
install(DIRECTORY
  scripts/
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package()
