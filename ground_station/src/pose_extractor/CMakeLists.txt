cmake_minimum_required(VERSION 3.8)
project(drone_swarm_mavlink_pose_extractor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Library: MAVLink frame parsing + NED→ENU math — no ROS node, fully testable.
add_library(mavlink_frame_parser src/mavlink_frame_parser.cpp)
target_include_directories(mavlink_frame_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable: ROS 2 pose extractor node
add_executable(mavlink_pose_extractor_node
  src/mavlink_pose_extractor_node.cpp
  src/main.cpp
)
target_include_directories(mavlink_pose_extractor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mavlink_pose_extractor_node mavlink_frame_parser)
ament_target_dependencies(mavlink_pose_extractor_node
  geometry_msgs
  rclcpp
  std_msgs
  tf2_ros
)

install(TARGETS mavlink_pose_extractor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mavlink_frame_parser
    test/test_mavlink_frame_parser.cpp
  )
  target_link_libraries(test_mavlink_frame_parser mavlink_frame_parser)
  target_include_directories(test_mavlink_frame_parser PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
