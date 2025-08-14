#include "motor.h"

const char *MAIN = "MAIN";

void app_main(void)
{
    esp_err_t ret = motor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(MAIN, "Motor initialization failed: %s", esp_err_to_name(ret));
        return;
    }
    
    motor_set_torque(255);
    motor_set_pwm_speed(255);
    
    while(1)
    {
        motor_step_clockwise();
    }
}
