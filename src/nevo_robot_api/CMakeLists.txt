cmake_minimum_required(VERSION 3.8)
project(nevo_robot_api)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# ══════════════════════════════════════════════════════════════════════════════
# Find all required packages FIRST
# ══════════════════════════════════════════════════════════════════════════════

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(CURL REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBZIP REQUIRED libzip)

# ══════════════════════════════════════════════════════════════════════════════
# Include directories
# ══════════════════════════════════════════════════════════════════════════════

include_directories(include)

# ══════════════════════════════════════════════════════════════════════════════
# SENSOR_MANAGER_NODE
# ══════════════════════════════════════════════════════════════════════════════

add_executable(sensor_manager_node
  src/sensor_manager_node.cpp
)

ament_target_dependencies(sensor_manager_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS sensor_manager_node
  DESTINATION lib/${PROJECT_NAME}
)


# ══════════════════════════════════════════════════════════════════════════════
# POSITION_SENSOR_NODE
# ══════════════════════════════════════════════════════════════════════════════

add_executable(position_sensor_node
  src/position_sensor_node.cpp
)

ament_target_dependencies(position_sensor_node
  rclcpp
  std_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

install(TARGETS position_sensor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ══════════════════════════════════════════════════════════════════════════════
# ROBOT_API_NODE — ✅ MAIN NODE WITH MAPPING + MAP SWITCHING SUPPORT
# ══════════════════════════════════════════════════════════════════════════════

add_executable(robot_api_node
  src/robot_api_node.cpp
  src/state_machine.cpp
)

target_include_directories(robot_api_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBZIP_INCLUDE_DIRS}
)

# ✅ Link dependencies AFTER add_executable
ament_target_dependencies(robot_api_node
  rclcpp
  rclcpp_action
  std_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
  nav2_msgs
  slam_toolbox
)

# ✅ Link CURL + libzip
target_link_libraries(robot_api_node
  CURL::libcurl
  ${LIBZIP_LIBRARIES}
)

install(TARGETS robot_api_node
  DESTINATION lib/${PROJECT_NAME}
)

# ══════════════════════════════════════════════════════════════════════════════
# SLAM_SUPERVISOR_NODE — ✅ MONITORS MAPPING COMPLETION
# ══════════════════════════════════════════════════════════════════════════════

add_executable(slam_supervisor_node
  src/slam_supervisor_node.cpp
)

target_include_directories(slam_supervisor_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(slam_supervisor_node
  rclcpp
  std_msgs
  nav_msgs
  geometry_msgs
)

install(TARGETS slam_supervisor_node
  DESTINATION lib/${PROJECT_NAME}
)

# ══════════════════════════════════════════════════════════════════════════════
# Install directories
# ══════════════════════════════════════════════════════════════════════════════

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# ══════════════════════════════════════════════════════════════════════════════
# Finalize ament package
# ══════════════════════════════════════════════════════════════════════════════

ament_package()