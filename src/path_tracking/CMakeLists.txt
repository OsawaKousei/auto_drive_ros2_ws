cmake_minimum_required(VERSION 3.8)
project(path_tracking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED) # path_server_node のため

ament_auto_find_build_dependencies()

# Path server node (既存)
ament_auto_add_executable(
  path_server_node
  src/path_server_node.cpp
)
ament_target_dependencies(path_server_node rclcpp nav_msgs geometry_msgs ament_index_cpp)

ament_auto_add_library(
  pure_pursuit
  src/pure_pursuit/pure_pursuit.cpp
  src/pure_pursuit/pure_pursuit_logic.cpp
)

rclcpp_components_register_nodes(
  pure_pursuit "pure_pursuit::PurePursuit"
)

ament_auto_add_executable(
  pure_pursuit_node
  src/pure_pursuit/pure_pursuit_node.cpp
)

install(
  DIRECTORY
  path
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)

  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
