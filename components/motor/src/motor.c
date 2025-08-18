#include "motor.h"

static const char *TAG = "MOTOR";
static int current_step_delay_ms = MOTOR_DEFAULT_STEP_DELAY_MS;
static int current_motor_duty = MOTOR_DEFAULT_DUTY_CYCLE;
static volatile bool motor_moving = false;
static volatile bool emergency_stop_flag = false;
static volatile float target_velocity_dps = 0.0;
static TaskHandle_t motor_task_handle = NULL;

static const long int gpio_bit_mask = (1ULL << MOTOR_A1_PIN) | (1ULL << MOTOR_B1_PIN) |
                                      (1ULL << MOTOR_A2_PIN) | (1ULL << MOTOR_B2_PIN);

static esp_err_t motor_setup_ledc(void);
static esp_err_t motor_setup_gpio(void);
static void motor_control_task(void *pvParameters);

esp_err_t motor_init(void) {
    esp_err_t ret;
    
    ret = motor_setup_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup GPIO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = motor_setup_ledc();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup LEDC: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_1, current_motor_duty);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_1);
    
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_2, current_motor_duty);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_2);
    
    motor_stop();
    
    xTaskCreate(motor_control_task, "motor_control", 4096, NULL, 3, &motor_task_handle);
    
    ESP_LOGI(TAG, "Motor initialized successfully (open-loop control)");
    ESP_LOGI(TAG, "Motor controls arm rotation without encoder feedback");
    return ESP_OK;
}

static esp_err_t motor_setup_gpio(void) {
    gpio_config_t gpio_conf = {
        .pin_bit_mask = gpio_bit_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    
    return gpio_config(&gpio_conf);
}

static esp_err_t motor_setup_ledc(void) {
    esp_err_t ret;
    
    ledc_timer_config_t ledc_timer = {
        .speed_mode = MOTOR_LEDC_MODE,
        .timer_num = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_DUTY_RES,
        .freq_hz = MOTOR_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ret = ledc_timer_config(&ledc_timer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_1,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_PWM_PIN_1,
        .duty = 0,
        .hpoint = 0
    };
    
    ret = ledc_channel_config(&ledc_channel_1);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ledc_channel_config_t ledc_channel_2 = {
        .speed_mode = MOTOR_LEDC_MODE,
        .channel = MOTOR_LEDC_CHANNEL_2,
        .timer_sel = MOTOR_LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MOTOR_PWM_PIN_2,
        .duty = 0,
        .hpoint = 0
    };
    
    return ledc_channel_config(&ledc_channel_2);
}

void motor_set_speed_rpm(float rpm) {
    if (rpm <= 0) {
        ESP_LOGW(TAG, "Invalid RPM value: %.2f. Must be positive.", rpm);
        return;
    }
    
    if (rpm > 180) {
        ESP_LOGW(TAG, "RPM %.2f exceeds maximum of 180. Clamping to 180.", rpm);
        rpm = 180;
    }
    
    int delay_ms = (int)(60000.0 / (rpm * 200.0));
    
    if (delay_ms < 1) delay_ms = 1;
    
    current_step_delay_ms = delay_ms;
    ESP_LOGD(TAG, "Motor speed set to %.2f RPM (delay: %d ms)", rpm, delay_ms);
}

void motor_set_step_delay(int delay_ms) {
    if (delay_ms < 1) {
        ESP_LOGW(TAG, "Invalid delay: %d ms. Must be at least 1 ms.", delay_ms);
        return;
    }
    
    current_step_delay_ms = delay_ms;
    float rpm = motor_calculate_rpm_from_delay(delay_ms);
    ESP_LOGD(TAG, "Step delay set to %d ms (%.2f RPM)", delay_ms, rpm);
}

void motor_set_torque(int duty_cycle) {
    if (duty_cycle < 0 || duty_cycle > 255) {
        ESP_LOGW(TAG, "Invalid duty cycle: %d. Must be 0-255.", duty_cycle);
        return;
    }
    
    current_motor_duty = duty_cycle;
    
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_1, duty_cycle);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_1);
    
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_2, duty_cycle);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_2);
    
    float torque_percent = (duty_cycle / 255.0) * 100.0;
    ESP_LOGD(TAG, "Motor torque set to %d/255 (%.1f%%)", duty_cycle, torque_percent);
}

void motor_set_pwm_speed(int duty_cycle) {
    if (duty_cycle < 0 || duty_cycle > 255) {
        ESP_LOGW(TAG, "Invalid PWM duty cycle: %d. Must be 0-255.", duty_cycle);
        return;
    }
    
    motor_set_torque(duty_cycle);
    
    if (duty_cycle < 50) {
        current_step_delay_ms = 50;
    } else if (duty_cycle < 100) {
        current_step_delay_ms = 30;
    } else if (duty_cycle < 150) {
        current_step_delay_ms = 20;
    } else {
        current_step_delay_ms = 17;
    }
    
    ESP_LOGD(TAG, "PWM speed control: duty=%d, delay=%dms", duty_cycle, current_step_delay_ms);
}

void motor_step_clockwise(void) {
    // ESP_LOGD(TAG, "Clock wise spin");  // Disabled to prevent output flooding
    
    gpio_set_level(MOTOR_A1_PIN, 1);
    gpio_set_level(MOTOR_B1_PIN, 0);
    gpio_set_level(MOTOR_A2_PIN, 1);
    gpio_set_level(MOTOR_B2_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
    
    gpio_set_level(MOTOR_A1_PIN, 0);
    gpio_set_level(MOTOR_B1_PIN, 1);
    gpio_set_level(MOTOR_A2_PIN, 1);
    gpio_set_level(MOTOR_B2_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
    
    gpio_set_level(MOTOR_A1_PIN, 0);
    gpio_set_level(MOTOR_B1_PIN, 1);
    gpio_set_level(MOTOR_A2_PIN, 0);
    gpio_set_level(MOTOR_B2_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
    
    gpio_set_level(MOTOR_A1_PIN, 1);
    gpio_set_level(MOTOR_B1_PIN, 0);
    gpio_set_level(MOTOR_A2_PIN, 0);
    gpio_set_level(MOTOR_B2_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
}

void motor_step_counter_clockwise(void) {
    // ESP_LOGD(TAG, "Counter clock wise spin");  // Disabled to prevent output flooding
    
    // Clockwise sequence reversed for counter-clockwise
    // Step 4 of clockwise becomes step 1 of counter-clockwise
    gpio_set_level(MOTOR_A1_PIN, 1);
    gpio_set_level(MOTOR_B1_PIN, 0);
    gpio_set_level(MOTOR_A2_PIN, 0);
    gpio_set_level(MOTOR_B2_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
    
    // Step 3 of clockwise becomes step 2 of counter-clockwise  
    gpio_set_level(MOTOR_A1_PIN, 0);
    gpio_set_level(MOTOR_B1_PIN, 1);
    gpio_set_level(MOTOR_A2_PIN, 0);
    gpio_set_level(MOTOR_B2_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
    
    // Step 2 of clockwise becomes step 3 of counter-clockwise
    gpio_set_level(MOTOR_A1_PIN, 0);
    gpio_set_level(MOTOR_B1_PIN, 1);
    gpio_set_level(MOTOR_A2_PIN, 1);
    gpio_set_level(MOTOR_B2_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
    
    // Step 1 of clockwise becomes step 4 of counter-clockwise
    gpio_set_level(MOTOR_A1_PIN, 1);
    gpio_set_level(MOTOR_B1_PIN, 0);
    gpio_set_level(MOTOR_A2_PIN, 1);
    gpio_set_level(MOTOR_B2_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(current_step_delay_ms));
}

void motor_stop(void) {
    gpio_set_level(MOTOR_A1_PIN, 0);
    gpio_set_level(MOTOR_B1_PIN, 0);
    gpio_set_level(MOTOR_A2_PIN, 0);
    gpio_set_level(MOTOR_B2_PIN, 0);
}

int motor_get_current_delay(void) {
    return current_step_delay_ms;
}

int motor_get_current_duty(void) {
    return current_motor_duty;
}

float motor_calculate_rpm_from_delay(int delay_ms) {
    return 60000.0 / (delay_ms * 200.0);
}

void motor_set_velocity_control(float velocity_dps) {
    if (emergency_stop_flag) {
        return;
    }
    
    if (fabs(velocity_dps) < 0.1f) {
        motor_moving = false;
        motor_stop();
        return;
    }
    
    float rpm = fabs(velocity_dps) / 6.0f;
    motor_set_speed_rpm(rpm);
    motor_moving = true;
}

void motor_move_to_angle(float target_angle_deg) {
    ESP_LOGI(TAG, "Moving to angle: %.2f degrees", target_angle_deg);
    motor_moving = true;
}

bool motor_is_moving(void) {
    return motor_moving && !emergency_stop_flag;
}

void motor_set_arm_velocity(float velocity_dps) {
    if (emergency_stop_flag) {
        return;
    }
    
    target_velocity_dps = velocity_dps;
    
    if (fabs(velocity_dps) < 0.1f) {
        motor_moving = false;
        ESP_LOGD(TAG, "Arm velocity too low, stopping motor");
        return;
    }
    
    // Convert degrees per second to RPM and set motor speed
    float rpm = fabs(velocity_dps) / 6.0f;
    motor_set_speed_rpm(rpm);
    motor_moving = true;
    
    // Direction control based on velocity sign
    if (velocity_dps > 0) {
        ESP_LOGD(TAG, "Arm velocity: %.2f dps (clockwise)", velocity_dps);
    } else {
        ESP_LOGD(TAG, "Arm velocity: %.2f dps (counter-clockwise)", velocity_dps);
    }
}

void motor_emergency_stop(void) {
    emergency_stop_flag = true;
    motor_moving = false;
    motor_stop();
    
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_1, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_1);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_2, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CHANNEL_2);
    
    ESP_LOGW(TAG, "EMERGENCY STOP ACTIVATED!");
}

static void motor_control_task(void *pvParameters) {
    while (1) {
        if (emergency_stop_flag || !motor_moving) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        if (fabs(target_velocity_dps) < 0.1f) {
            motor_moving = false;
            motor_stop();
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        
        // Execute steps based on direction
        if (target_velocity_dps > 0) {
            motor_step_clockwise();
        } else {
            motor_step_counter_clockwise();
        }
    }
}