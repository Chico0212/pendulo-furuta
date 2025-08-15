idf_component_register(SRCS "src/encoder.c"
                       INCLUDE_DIRS "inc"
                       REQUIRES "driver" "freertos" "esp_timer")