cmake_minimum_required(VERSION 3.8)
project(nevo_ws_bridge)

# =========================
# C++ SETTINGS
# =========================
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

# =========================
# DEPENDENCIES
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenSSL REQUIRED)

include(FetchContent)

# =========================
# IXWebSocket
# =========================
set(USE_TLS ON CACHE BOOL "" FORCE)
set(USE_OPEN_SSL ON CACHE BOOL "" FORCE)
set(USE_MBED_TLS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  ixwebsocket
  GIT_REPOSITORY https://github.com/machinezone/IXWebSocket.git
  GIT_TAG master
)

FetchContent_MakeAvailable(ixwebsocket)

# =========================
# INCLUDE
# =========================
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

# =========================
# LIB: ws_client
# =========================
add_library(ws_client
  src/ws_client.cpp
)

target_include_directories(ws_client PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
)

ament_target_dependencies(ws_client
  rclcpp
  std_msgs
)

target_link_libraries(ws_client
  ixwebsocket::ixwebsocket
  OpenSSL::SSL
  OpenSSL::Crypto
)

# =========================
# LIB: ws_server
# =========================
add_library(ws_server
  src/ws_server.cpp
)

target_include_directories(ws_server PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
)

ament_target_dependencies(ws_server
  rclcpp
  std_msgs
)

target_link_libraries(ws_server
  ixwebsocket::ixwebsocket
)

# =========================
# EXEC: ws_bridge_node
# =========================
add_executable(ws_bridge_node
  src/ws_bridge_node.cpp
)

target_include_directories(ws_bridge_node PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
)

ament_target_dependencies(ws_bridge_node
  rclcpp
  std_msgs
)

target_link_libraries(ws_bridge_node
  ws_client
  ws_server
  ixwebsocket::ixwebsocket
)

# =========================
# EXEC: cloud_upload 🔥
# =========================
add_executable(cloud_upload
  src/cloud_upload.cpp
)

target_include_directories(cloud_upload PUBLIC
  $<BUILD_INTERFACE:${INCLUDE_DIR}>
)

ament_target_dependencies(cloud_upload
  rclcpp
  std_msgs
)

target_link_libraries(cloud_upload
  ws_client
  ws_server
  ixwebsocket::ixwebsocket
)

# =========================
# INSTALL
# =========================
install(TARGETS
  ws_bridge_node
  cloud_upload
  ws_client
  ws_server
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()