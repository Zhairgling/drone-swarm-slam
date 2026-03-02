idf_component_register(
    SRCS "mavlink_bridge.c" "mavlink_parser.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES driver micro_ros_espidf_component
)
