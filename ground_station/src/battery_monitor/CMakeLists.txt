cmake_minimum_required(VERSION 3.8)
project(drone_swarm_battery_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Library: MAVLink SYS_STATUS parsing — no ROS node, fully testable.
add_library(mavlink_battery_parser src/mavlink_battery_parser.cpp)
target_include_directories(mavlink_battery_parser PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable: ROS 2 battery monitor node
add_executable(battery_monitor_node
  src/battery_monitor_node.cpp
  src/main.cpp
)
target_include_directories(battery_monitor_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(battery_monitor_node mavlink_battery_parser)
ament_target_dependencies(battery_monitor_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS battery_monitor_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mavlink_battery_parser
    test/test_mavlink_battery_parser.cpp
  )
  target_link_libraries(test_mavlink_battery_parser mavlink_battery_parser)
  target_include_directories(test_mavlink_battery_parser PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
