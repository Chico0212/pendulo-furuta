#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ENCODER_CHANNEL_A_PIN GPIO_NUM_32
#define ENCODER_CHANNEL_B_PIN GPIO_NUM_33

#define ENCODER_PPR 400
#define ENCODER_COUNTS_PER_REV (ENCODER_PPR * 4)

#define ENCODER_SPEED_CALC_INTERVAL_US 100000

typedef enum {
    ENCODER_DIRECTION_CLOCKWISE,
    ENCODER_DIRECTION_COUNTER_CLOCKWISE,
    ENCODER_DIRECTION_UNKNOWN
} encoder_direction_t;

typedef struct {
    int64_t position;
    float speed_rpm;
    encoder_direction_t direction;
    uint32_t last_pulse_time_us;
} encoder_data_t;

esp_err_t encoder_init(void);

void encoder_reset_position(void);

int64_t encoder_get_position(void);

float encoder_get_speed_rpm(void);

encoder_direction_t encoder_get_direction(void);

encoder_data_t encoder_get_data(void);

float encoder_position_to_degrees(int64_t position);

float encoder_position_to_radians(int64_t position);

int64_t encoder_degrees_to_position(float degrees);

int64_t encoder_radians_to_position(float radians);

void encoder_debug_pins(void);

#ifdef __cplusplus
}
#endif

#endif // ENCODER_H