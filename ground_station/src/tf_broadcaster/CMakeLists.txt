cmake_minimum_required(VERSION 3.8)
project(drone_swarm_tf_broadcaster)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

# Library: pure transform logic (no ROS spin, testable standalone)
add_library(drone_tf_broadcaster_lib
  src/drone_tf_broadcaster.cpp
)
target_include_directories(drone_tf_broadcaster_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(drone_tf_broadcaster_lib
  geometry_msgs
  rclcpp
  tf2_ros
)

# Executable: ROS 2 node
add_executable(drone_tf_broadcaster_node
  src/main.cpp
)
target_include_directories(drone_tf_broadcaster_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(drone_tf_broadcaster_node drone_tf_broadcaster_lib)
ament_target_dependencies(drone_tf_broadcaster_node
  geometry_msgs
  rclcpp
  tf2_ros
)

install(TARGETS drone_tf_broadcaster_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_drone_tf_broadcaster
    test/test_drone_tf_broadcaster.cpp
  )
  target_link_libraries(test_drone_tf_broadcaster drone_tf_broadcaster_lib)
  target_include_directories(test_drone_tf_broadcaster PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_drone_tf_broadcaster
    geometry_msgs
    rclcpp
    tf2_ros
  )
endif()

ament_package()
