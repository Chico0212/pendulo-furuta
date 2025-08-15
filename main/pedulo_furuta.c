#include "motor.h"
#include "encoder.h"

const char *MAIN = "MAIN";

void app_main(void)
{
    esp_err_t ret = motor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(MAIN, "Motor initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    ret = encoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE(MAIN, "Encoder initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    motor_set_torque(255);
    motor_set_pwm_speed(255);
    
    int step_count = 0;
    encoder_reset_position();
    
    while(1)
    {
        // motor_step_clockwise();
        step_count++;
        
        if (step_count % 10 == 0) {
            encoder_debug_pins();
            encoder_data_t encoder_data = encoder_get_data();
            float position_degrees = encoder_position_to_degrees(encoder_data.position);
            
            ESP_LOGI(MAIN, "Encoder - Position: %lld counts (%.2f°), Speed: %.2f RPM, Direction: %s", 
                     encoder_data.position, 
                     position_degrees,
                     encoder_data.speed_rpm,
                     (encoder_data.direction == ENCODER_DIRECTION_CLOCKWISE) ? "CW" : 
                     (encoder_data.direction == ENCODER_DIRECTION_COUNTER_CLOCKWISE) ? "CCW" : "UNKNOWN");
        }

        // vTaskDelay(pdMS_TO_TICKS(1));
    }
}
