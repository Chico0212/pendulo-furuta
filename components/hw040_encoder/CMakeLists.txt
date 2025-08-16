idf_component_register(
    SRCS "src/hw040_encoder.c"
    INCLUDE_DIRS "inc"
    REQUIRES "driver" "esp_timer" "freertos"
)