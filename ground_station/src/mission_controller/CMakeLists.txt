cmake_minimum_required(VERSION 3.8)
project(drone_swarm_mission_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# Library: waypoint sequencing logic — no ROS dependency, fully testable
add_library(waypoint_manager
  src/waypoint_manager.cpp
)
target_include_directories(waypoint_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executable: ROS 2 mission controller node
add_executable(mission_controller_node
  src/mission_controller_node.cpp
  src/main.cpp
)
target_include_directories(mission_controller_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(mission_controller_node waypoint_manager)
ament_target_dependencies(mission_controller_node
  rclcpp
  geometry_msgs
)

install(TARGETS mission_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_waypoint_manager test/test_waypoint_manager.cpp)
  target_link_libraries(test_waypoint_manager waypoint_manager)
  target_include_directories(test_waypoint_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

ament_package()
