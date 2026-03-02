cmake_minimum_required(VERSION 3.8)
project(drone_swarm_pose_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rclcpp REQUIRED)

# Library: MAVLink encoding logic — pure C++, no ROS node, fully testable.
add_library(mavlink_encoder src/mavlink_encoder.cpp)
target_include_directories(mavlink_encoder PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mavlink_encoder geometry_msgs mavros_msgs)

# Executable: ROS 2 pose estimator node
add_executable(pose_estimator_node
  src/pose_estimator_node.cpp
  src/main.cpp
)
target_include_directories(pose_estimator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pose_estimator_node mavlink_encoder)
ament_target_dependencies(pose_estimator_node rclcpp geometry_msgs mavros_msgs)

install(TARGETS pose_estimator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mavlink_encoder test/test_mavlink_encoder.cpp)
  target_link_libraries(test_mavlink_encoder mavlink_encoder)
  target_include_directories(test_mavlink_encoder PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_mavlink_encoder geometry_msgs mavros_msgs)
endif()

ament_package()
