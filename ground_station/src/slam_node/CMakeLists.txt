cmake_minimum_required(VERSION 3.8)
project(drone_swarm_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

# Library: occupancy map logic (reusable, testable without ROS node)
add_library(occupancy_map
  src/occupancy_map.cpp
)
target_include_directories(occupancy_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(occupancy_map
  nav_msgs
  sensor_msgs
  std_msgs
)

# Executable: SLAM ROS 2 node
add_executable(slam_node_exec
  src/slam_node.cpp
  src/main.cpp
)
target_include_directories(slam_node_exec PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(slam_node_exec occupancy_map)
ament_target_dependencies(slam_node_exec
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_sensor_msgs
)
set_target_properties(slam_node_exec PROPERTIES OUTPUT_NAME slam_node)

install(TARGETS slam_node_exec
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_gtest(test_occupancy_map test/test_occupancy_map.cpp)
  target_link_libraries(test_occupancy_map occupancy_map)
  target_include_directories(test_occupancy_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_occupancy_map
    nav_msgs
    sensor_msgs
    std_msgs
  )

  ament_add_pytest_test(test_slam_node_launch
    test/test_slam_node_launch.py
    TIMEOUT 60
  )
endif()

ament_package()
