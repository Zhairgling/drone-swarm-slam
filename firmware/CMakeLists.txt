cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_LIST_DIR}/components
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(drone_swarm_firmware)

# DESIGN: micro-ROS libmicroros.a provides __atomic_*_8 symbols that conflict
# with ESP-IDF newlib on v5.3+. Allow multiple definitions to resolve the clash.
idf_build_set_property(LINK_OPTIONS "-Wl,--allow-multiple-definition" APPEND)
