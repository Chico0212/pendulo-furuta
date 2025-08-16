#include "furuta_sensors.h"
#include "encoder.h"
#include "hw040_encoder.h"
#include "esp_timer.h"

static const char *TAG = "FURUTA_SENSORS";

esp_err_t furuta_sensors_init(furuta_sensors_t *sensors) {
    if (sensors == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    sensors->config.arm_offset_rad = 0.0f;
    sensors->config.pendulum_offset_rad = 0.0f;
    sensors->config.arm_scale_factor = 1.0f;
    sensors->config.pendulum_scale_factor = 1.0f;
    sensors->config.arm_encoder_inverted = false;
    sensors->config.pendulum_encoder_inverted = false;
    sensors->config.velocity_filter_alpha = FURUTA_SENSOR_FILTER_ALPHA;
    
    furuta_sensors_reset_filters(sensors);
    
    sensors->initialized = true;
    
    ESP_LOGI(TAG, "Sensor fusion initialized");
    return ESP_OK;
}

void furuta_sensors_set_config(furuta_sensors_t *sensors, furuta_sensor_config_t *config) {
    if (sensors == NULL || config == NULL) {
        return;
    }
    
    sensors->config = *config;
    ESP_LOGI(TAG, "Sensor configuration updated");
}

furuta_sensor_data_t furuta_sensors_read_all(furuta_sensors_t *sensors) {
    furuta_sensor_data_t data = {0};
    
    if (sensors == NULL || !sensors->initialized) {
        data.data_valid = false;
        return data;
    }
    
    float current_time_s = esp_timer_get_time() / 1000000.0f;
    
    data.arm_angle_rad = furuta_sensors_read_arm_angle();
    data.pendulum_angle_rad = furuta_sensors_read_pendulum_angle();
    
    if (sensors->last_timestamp_s > 0.0f) {
        float dt = current_time_s - sensors->last_timestamp_s;
        
        if (dt > 0.0f && dt < 1.0f) {
            float arm_velocity_raw = (data.arm_angle_rad - sensors->last_arm_angle_rad) / dt;
            float pendulum_velocity_raw = (data.pendulum_angle_rad - sensors->last_pendulum_angle_rad) / dt;
            
            if (arm_velocity_raw > M_PI) {
                arm_velocity_raw -= 2.0f * M_PI;
            } else if (arm_velocity_raw < -M_PI) {
                arm_velocity_raw += 2.0f * M_PI;
            }
            
            if (pendulum_velocity_raw > M_PI) {
                pendulum_velocity_raw -= 2.0f * M_PI;
            } else if (pendulum_velocity_raw < -M_PI) {
                pendulum_velocity_raw += 2.0f * M_PI;
            }
            
            sensors->filtered_arm_velocity_rad_s = sensors->config.velocity_filter_alpha * 
                                                  sensors->filtered_arm_velocity_rad_s + 
                                                  (1.0f - sensors->config.velocity_filter_alpha) * arm_velocity_raw;
            
            sensors->filtered_pendulum_velocity_rad_s = sensors->config.velocity_filter_alpha * 
                                                       sensors->filtered_pendulum_velocity_rad_s + 
                                                       (1.0f - sensors->config.velocity_filter_alpha) * pendulum_velocity_raw;
        }
    }
    
    data.arm_velocity_rad_s = sensors->filtered_arm_velocity_rad_s;
    data.pendulum_velocity_rad_s = sensors->filtered_pendulum_velocity_rad_s;
    data.timestamp_s = current_time_s;
    data.data_valid = true;
    
    sensors->last_arm_angle_rad = data.arm_angle_rad;
    sensors->last_pendulum_angle_rad = data.pendulum_angle_rad;
    sensors->last_timestamp_s = current_time_s;
    
    return data;
}

float furuta_sensors_read_arm_angle(void) {
    // READ ARM ENCODER (HW-040 encoder)
    hw040_data_t hw040_data = hw040_encoder_get_data();
    float raw_angle_rad = hw040_position_to_radians(hw040_data.position);
    
    return raw_angle_rad;
}

float furuta_sensors_read_pendulum_angle(void) {
    // READ ACTUAL PENDULUM ENCODER (A38 encoder)
    encoder_data_t encoder_data = encoder_get_data();
    float raw_angle_rad = encoder_position_to_radians(encoder_data.position);
    
    return raw_angle_rad;
}

void furuta_sensors_calibrate_offsets(furuta_sensors_t *sensors) {
    if (sensors == NULL) {
        return;
    }
    
    const int num_samples = 100;
    float arm_sum = 0.0f;
    float pendulum_sum = 0.0f;
    
    ESP_LOGI(TAG, "Calibrating dual encoder setup...");
    ESP_LOGI(TAG, "  - Pendulum should hang down (0°)");
    ESP_LOGI(TAG, "  - Arm should be at reference position");
    
    for (int i = 0; i < num_samples; i++) {
        arm_sum += furuta_sensors_read_arm_angle();
        pendulum_sum += furuta_sensors_read_pendulum_angle();
        
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    
    sensors->config.arm_offset_rad = arm_sum / num_samples;
    sensors->config.pendulum_offset_rad = pendulum_sum / num_samples - M_PI;
    
    ESP_LOGI(TAG, "Dual encoder calibration complete:");
    ESP_LOGI(TAG, "  Arm offset (HW-040): %.4f rad (%.2f°)", 
             sensors->config.arm_offset_rad, 
             sensors->config.arm_offset_rad * 180.0f / M_PI);
    ESP_LOGI(TAG, "  Pendulum offset (A38): %.4f rad (%.2f°)", 
             sensors->config.pendulum_offset_rad, 
             sensors->config.pendulum_offset_rad * 180.0f / M_PI);
    ESP_LOGI(TAG, "  System ready with dual encoder feedback");
}

bool furuta_sensors_is_data_valid(furuta_sensor_data_t *data) {
    if (data == NULL) {
        return false;
    }
    
    if (!data->data_valid) {
        return false;
    }
    
    if (isnan(data->arm_angle_rad) || isnan(data->pendulum_angle_rad) ||
        isnan(data->arm_velocity_rad_s) || isnan(data->pendulum_velocity_rad_s)) {
        return false;
    }
    
    if (fabs(data->arm_velocity_rad_s) > 50.0f || fabs(data->pendulum_velocity_rad_s) > 50.0f) {
        return false;
    }
    
    return true;
}

void furuta_sensors_reset_filters(furuta_sensors_t *sensors) {
    if (sensors == NULL) {
        return;
    }
    
    sensors->last_arm_angle_rad = 0.0f;
    sensors->last_pendulum_angle_rad = 0.0f;
    sensors->filtered_arm_velocity_rad_s = 0.0f;
    sensors->filtered_pendulum_velocity_rad_s = 0.0f;
    sensors->last_timestamp_s = 0.0f;
    
    ESP_LOGI(TAG, "Sensor filters reset");
}