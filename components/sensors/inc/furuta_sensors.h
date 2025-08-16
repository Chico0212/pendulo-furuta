#ifndef FURUTA_SENSORS_H
#define FURUTA_SENSORS_H

#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "encoder.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FURUTA_SENSOR_UPDATE_FREQUENCY_HZ 100.0f
#define FURUTA_SENSOR_FILTER_ALPHA 0.95f

typedef struct {
    float arm_angle_rad;
    float pendulum_angle_rad;
    float arm_velocity_rad_s;
    float pendulum_velocity_rad_s;
    float timestamp_s;
    bool data_valid;
} furuta_sensor_data_t;

typedef struct {
    float arm_offset_rad;
    float pendulum_offset_rad;
    float arm_scale_factor;
    float pendulum_scale_factor;
    bool arm_encoder_inverted;
    bool pendulum_encoder_inverted;
    float velocity_filter_alpha;
} furuta_sensor_config_t;

typedef struct {
    furuta_sensor_config_t config;
    float last_arm_angle_rad;
    float last_pendulum_angle_rad;
    float filtered_arm_velocity_rad_s;
    float filtered_pendulum_velocity_rad_s;
    float last_timestamp_s;
    bool initialized;
} furuta_sensors_t;

esp_err_t furuta_sensors_init(furuta_sensors_t *sensors);

void furuta_sensors_set_config(furuta_sensors_t *sensors, furuta_sensor_config_t *config);

furuta_sensor_data_t furuta_sensors_read_all(furuta_sensors_t *sensors);

float furuta_sensors_read_arm_angle(void);

float furuta_sensors_read_pendulum_angle(void);

void furuta_sensors_calibrate_offsets(furuta_sensors_t *sensors);

bool furuta_sensors_is_data_valid(furuta_sensor_data_t *data);

void furuta_sensors_reset_filters(furuta_sensors_t *sensors);

#ifdef __cplusplus
}
#endif

#endif // FURUTA_SENSORS_H