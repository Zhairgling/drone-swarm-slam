cmake_minimum_required(VERSION 3.8)
project(drone_swarm_tof_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Library: ToF data generation logic (reusable, testable)
add_library(tof_data_generator
  src/tof_data_generator.cpp
)
target_include_directories(tof_data_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(tof_data_generator
  sensor_msgs
  std_msgs
)

# Executable: ROS 2 simulator node
add_executable(tof_simulator_node
  src/tof_simulator_node.cpp
  src/main.cpp
)
target_include_directories(tof_simulator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(tof_simulator_node tof_data_generator)
ament_target_dependencies(tof_simulator_node
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS tof_simulator_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_tof_data_generator test/test_tof_data_generator.cpp)
  target_link_libraries(test_tof_data_generator tof_data_generator)
  target_include_directories(test_tof_data_generator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
  ament_target_dependencies(test_tof_data_generator
    sensor_msgs
    std_msgs
  )
endif()

ament_package()
