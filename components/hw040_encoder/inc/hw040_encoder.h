#ifndef HW040_ENCODER_H
#define HW040_ENCODER_H

#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// HW-040 encoder pins for arm measurement
#define HW040_CHANNEL_A_PIN GPIO_NUM_4
#define HW040_CHANNEL_B_PIN GPIO_NUM_5

// HW-040 specifications
#define HW040_PPR 20  // 20 pulses per revolution
#define HW040_COUNTS_PER_REV (HW040_PPR * 4)  // 80 counts per revolution with quadrature

#define HW040_SPEED_CALC_INTERVAL_US 100000  // 100ms

typedef enum {
    HW040_DIRECTION_CLOCKWISE,
    HW040_DIRECTION_COUNTER_CLOCKWISE,
    HW040_DIRECTION_UNKNOWN
} hw040_direction_t;

typedef struct {
    int64_t position;
    float speed_rpm;
    hw040_direction_t direction;
    uint32_t last_pulse_time_us;
} hw040_data_t;

// Initialization and control functions
esp_err_t hw040_encoder_init(void);

void hw040_encoder_reset_position(void);

// Data reading functions
int64_t hw040_encoder_get_position(void);

float hw040_encoder_get_speed_rpm(void);

hw040_direction_t hw040_encoder_get_direction(void);

hw040_data_t hw040_encoder_get_data(void);

// Conversion functions
float hw040_position_to_degrees(int64_t position);

float hw040_position_to_radians(int64_t position);

int64_t hw040_degrees_to_position(float degrees);

int64_t hw040_radians_to_position(float radians);

// Get arm angle in degrees (0° = reference position)
float hw040_get_arm_angle_degrees(void);

// Get arm angle in radians (0 = reference position)
float hw040_get_arm_angle_radians(void);

#ifdef __cplusplus
}
#endif

#endif // HW040_ENCODER_H