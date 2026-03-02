idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES micro_ros_espidf_component mavlink_bridge ov2640 esp_timer
)
