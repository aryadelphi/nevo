cmake_minimum_required(VERSION 3.8)
project(nevo_bot)

# ===============================
# Compiler options
# ===============================
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===============================
# Dependencies (SEMUA DIPERTAHANKAN)
# ===============================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)


# ===============================
# C++ Executables
# ===============================
include_directories(include)

add_executable(publish_initial_pose src/publish_initial_pose.cpp)
ament_target_dependencies(publish_initial_pose rclcpp geometry_msgs lifecycle_msgs)

add_executable(safety_fail_node src/safety_fail_node.cpp)
ament_target_dependencies(safety_fail_node rclcpp geometry_msgs std_msgs)

install(TARGETS
  publish_initial_pose
  safety_fail_node
  DESTINATION lib/${PROJECT_NAME}
)
# ===============================
# Install other resources
# ===============================
install(
  DIRECTORY
    config
    description
    launch
    meshes
    models
    maps
    worlds
    scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/exploration_save_node.py
  scripts/map_watcher_node.py
  scripts/ina226_bridge_node.py
  scripts/imu_publisher.py
  scripts/slam_pose_correction.py
  scripts/smart_dock_final.py
  scripts/tag_pose_bridge.py
  scripts/archive_tests/tag_pose_bridge_sim.py   # used by launch/docking_sim.launch.py
  scripts/smart_dock.py
  scripts/pure_dock.py
  scripts/full_dock.py
  scripts/simple_dock.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===============================
# Testing
# ===============================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
