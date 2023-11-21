cmake_minimum_required(VERSION 3.8)
project(laser_copilot)

add_compile_options( -std=c++2a -O3)

# find dependencies
find_package (ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies ()

find_package(Eigen3 REQUIRED)

ament_auto_add_library(obj_dist_component SHARED src/obj_dist_node.cpp)
rclcpp_components_register_nodes(obj_dist_component 
  "laser_copilot::obj_dist"
)

ament_auto_add_library(safe_fly_controller_component SHARED src/safe_fly_controller.cc)
target_link_libraries(safe_fly_controller_component Eigen3::Eigen)
rclcpp_components_register_nodes(safe_fly_controller_component
  "laser_copilot::safe_fly_controller"
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
