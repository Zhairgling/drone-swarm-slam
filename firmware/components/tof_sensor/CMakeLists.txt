idf_component_register(
    SRC_DIRS "src"
    INCLUDE_DIRS "include"
    REQUIRES driver
    PRIV_REQUIRES vl53l8cx esp_timer
)
