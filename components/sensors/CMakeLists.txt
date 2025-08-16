idf_component_register(SRCS "src/furuta_sensors.c"
                       INCLUDE_DIRS "inc"
                       REQUIRES "encoder" "hw040_encoder" "esp_timer" "freertos")