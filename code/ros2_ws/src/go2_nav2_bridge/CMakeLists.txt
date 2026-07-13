cmake_minimum_required(VERSION 3.8)
project(go2_nav2_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

option(BUILD_NAV2_STRAIGHT_PATH_CLIENT
  "Build the Nav2 FollowPath/ComputePathToPose demo client. Disabled by default for ROS 2 Foxy compatibility."
  OFF)

set(UNITREE_SDK2_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../unitree_sdk2"
  CACHE PATH "Path to unitree_sdk2")
set(UNITREE_SDK2_ARCH
  "${CMAKE_SYSTEM_PROCESSOR}"
  CACHE STRING "Unitree SDK architecture directory")

find_library(UNITREE_SDK2_LIB
  NAMES unitree_sdk2
  PATHS "${UNITREE_SDK2_DIR}/lib/${UNITREE_SDK2_ARCH}"
  NO_DEFAULT_PATH)
find_library(UNITREE_DDSC_LIB
  NAMES ddsc
  PATHS "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  NO_DEFAULT_PATH)
find_library(UNITREE_DDSCXX_LIB
  NAMES ddscxx
  PATHS "${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  NO_DEFAULT_PATH)

if(NOT UNITREE_SDK2_LIB OR NOT UNITREE_DDSC_LIB OR NOT UNITREE_DDSCXX_LIB)
  message(FATAL_ERROR "Unitree SDK2 libraries were not found under ${UNITREE_SDK2_DIR}; set UNITREE_SDK2_DIR and UNITREE_SDK2_ARCH.")
endif()

add_executable(go2_nav2_bridge src/go2_nav2_bridge.cpp)
target_include_directories(go2_nav2_bridge PRIVATE
  "${UNITREE_SDK2_DIR}/include"
  "${UNITREE_SDK2_DIR}/thirdparty/include"
  "${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx")
ament_target_dependencies(go2_nav2_bridge
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros)
target_link_libraries(go2_nav2_bridge
  "${UNITREE_SDK2_LIB}"
  "${UNITREE_DDSCXX_LIB}"
  "${UNITREE_DDSC_LIB}")
set_target_properties(go2_nav2_bridge PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_DIR}/lib/${UNITREE_SDK2_ARCH};${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  INSTALL_RPATH "${UNITREE_SDK2_DIR}/lib/${UNITREE_SDK2_ARCH};${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  LINK_FLAGS "-Wl,--disable-new-dtags")

if(BUILD_NAV2_STRAIGHT_PATH_CLIENT)
  add_executable(nav2_straight_path_client src/nav2_straight_path_client.cpp)
  ament_target_dependencies(nav2_straight_path_client
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
    nav2_msgs
    tf2
    tf2_ros)
endif()

add_executable(go2_direct_straight_controller src/go2_direct_straight_controller.cpp)
ament_target_dependencies(go2_direct_straight_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs)

add_executable(go2_waypoint_follower src/go2_waypoint_follower.cpp)
ament_target_dependencies(go2_waypoint_follower
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2)

add_executable(go2_pose_path_visualizer src/go2_pose_path_visualizer.cpp)
ament_target_dependencies(go2_pose_path_visualizer
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs)

add_executable(go2_voxel_map_visualizer src/go2_voxel_map_visualizer.cpp)
target_include_directories(go2_voxel_map_visualizer PRIVATE
  "${UNITREE_SDK2_DIR}/include"
  "${UNITREE_SDK2_DIR}/thirdparty/include"
  "${UNITREE_SDK2_DIR}/thirdparty/include/ddscxx")
ament_target_dependencies(go2_voxel_map_visualizer
  rclcpp
  sensor_msgs
  std_msgs
  visualization_msgs)
target_link_libraries(go2_voxel_map_visualizer
  "${UNITREE_SDK2_LIB}"
  "${UNITREE_DDSCXX_LIB}"
  "${UNITREE_DDSC_LIB}")
set_target_properties(go2_voxel_map_visualizer PROPERTIES
  BUILD_RPATH "${UNITREE_SDK2_DIR}/lib/${UNITREE_SDK2_ARCH};${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  INSTALL_RPATH "${UNITREE_SDK2_DIR}/lib/${UNITREE_SDK2_ARCH};${UNITREE_SDK2_DIR}/thirdparty/lib/${UNITREE_SDK2_ARCH}"
  LINK_FLAGS "-Wl,--disable-new-dtags")

set(GO2_NAV2_BRIDGE_TARGETS
  go2_nav2_bridge
  go2_direct_straight_controller
  go2_waypoint_follower
  go2_pose_path_visualizer
  go2_voxel_map_visualizer)

if(BUILD_NAV2_STRAIGHT_PATH_CLIENT)
  list(APPEND GO2_NAV2_BRIDGE_TARGETS nav2_straight_path_client)
endif()

install(TARGETS
  ${GO2_NAV2_BRIDGE_TARGETS}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params rviz config
  DESTINATION share/${PROJECT_NAME})

ament_package()
