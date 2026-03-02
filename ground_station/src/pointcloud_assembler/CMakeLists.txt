cmake_minimum_required(VERSION 3.8)
project(drone_swarm_pointcloud_assembler)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Library: pure assembler logic (no ROS spin, testable standalone)
add_library(pointcloud_assembler_lib
  src/pointcloud_assembler.cpp
)
target_include_directories(pointcloud_assembler_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(pointcloud_assembler_lib
  sensor_msgs
  std_msgs
)

# Executable: ROS 2 node
add_executable(pointcloud_assembler_node
  src/pointcloud_assembler_node.cpp
  src/main.cpp
)
target_include_directories(pointcloud_assembler_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(pointcloud_assembler_node pointcloud_assembler_lib)
ament_target_dependencies(pointcloud_assembler_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS pointcloud_assembler_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_pointcloud_assembler
    test/test_pointcloud_assembler.cpp
  )
  target_link_libraries(test_pointcloud_assembler pointcloud_assembler_lib)
  target_include_directories(test_pointcloud_assembler PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_pointcloud_assembler
    sensor_msgs
    std_msgs
  )
endif()

ament_package()
