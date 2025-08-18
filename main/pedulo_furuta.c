#include "encoder.h"
#include "hw040_encoder.h"
#include "motor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>

#define CONTROL_FREQUENCY_HZ 50
#define CONTROL_PERIOD_MS (1000 / CONTROL_FREQUENCY_HZ)

#define SWING_UP_THRESHOLD_DEG 30.0
#define BALANCE_THRESHOLD_DEG 15.0

#define KP_BALANCE 150.0
#define KD_BALANCE 10.0
#define KP_SWING_UP 50.0

typedef enum {
    FURUTA_STATE_IDLE,
    FURUTA_STATE_SWING_UP,
    FURUTA_STATE_BALANCE
} furuta_state_t;

static furuta_state_t current_state = FURUTA_STATE_IDLE;
static float previous_pendulum_angle = 0.0;
static uint32_t last_control_time = 0;

float normalize_angle(float angle) {
    angle = fmod(angle + 180.0, 360.0) - 180.0;
    if (angle < -180.0) angle += 360.0;
    return angle;
}

float calculate_pendulum_velocity(float current_angle) {
    static uint32_t last_time = 0;
    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    if (last_time == 0) {
        last_time = current_time;
        return 0.0;
    }
    
    float dt = (current_time - last_time) / 1000.0;
    if (dt <= 0) return 0.0;
    
    float velocity = (current_angle - previous_pendulum_angle) / dt;
    
    last_time = current_time;
    previous_pendulum_angle = current_angle;
    
    return velocity;
}

float swing_up_control(float pendulum_angle, float pendulum_velocity) {
    float energy_error = cos(pendulum_angle * M_PI / 180.0) + 1.0;
    float control_signal = KP_SWING_UP * energy_error * pendulum_velocity;
    
    if (control_signal > 100.0) control_signal = 100.0;
    if (control_signal < -100.0) control_signal = -100.0;
    
    return control_signal;
}

float balance_control(float pendulum_angle, float pendulum_velocity) {
    float angle_error = normalize_angle(pendulum_angle - 180.0);
    float control_signal = KP_BALANCE * angle_error + KD_BALANCE * pendulum_velocity;
    
    if (control_signal > 200.0) control_signal = 200.0;
    if (control_signal < -200.0) control_signal = -200.0;
    
    return control_signal;
}

void furuta_control_task(void *pvParameters) {
    ESP_LOGI("FURUTA", "Sistema de controle iniciado");
    
    while (1) {
        encoder_data_t encoder_data = encoder_get_data();
        float pendulum_angle = encoder_get_pendulum_angle_degrees();
        float pendulum_velocity = calculate_pendulum_velocity(pendulum_angle);
        
        float upright_error = fabs(normalize_angle(pendulum_angle - 180.0));
        float control_signal = 0.0;
        
        switch (current_state) {
            case FURUTA_STATE_IDLE:
                if (upright_error > SWING_UP_THRESHOLD_DEG) {
                    current_state = FURUTA_STATE_SWING_UP;
                    ESP_LOGI("FURUTA", "Mudando para SWING_UP");
                }
                motor_stop();
                break;
                
            case FURUTA_STATE_SWING_UP:
                if (upright_error < BALANCE_THRESHOLD_DEG) {
                    current_state = FURUTA_STATE_BALANCE;
                    ESP_LOGI("FURUTA", "Mudando para BALANCE");
                } else {
                    control_signal = swing_up_control(pendulum_angle, pendulum_velocity);
                    if (control_signal > 0) {
                        motor_set_arm_velocity(control_signal);
                    } else {
                        motor_set_arm_velocity(control_signal);
                    }
                }
                break;
                
            case FURUTA_STATE_BALANCE:
                if (upright_error > SWING_UP_THRESHOLD_DEG) {
                    current_state = FURUTA_STATE_SWING_UP;
                    ESP_LOGI("FURUTA", "Perdeu equilíbrio, voltando para SWING_UP");
                } else {
                    control_signal = balance_control(pendulum_angle, pendulum_velocity);
                    motor_set_arm_velocity(control_signal);
                }
                break;
        }
        
        if (xTaskGetTickCount() * portTICK_PERIOD_MS - last_control_time >= 1000) {
            ESP_LOGI("FURUTA", "Estado: %s | Ângulo: %.1f° | Vel: %.1f°/s | Controle: %.1f", 
                     (current_state == FURUTA_STATE_IDLE) ? "IDLE" :
                     (current_state == FURUTA_STATE_SWING_UP) ? "SWING_UP" : "BALANCE",
                     pendulum_angle, pendulum_velocity, control_signal);
            last_control_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
        
        vTaskDelay(pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    }
}

void app_main() {
    ESP_LOGI("MAIN", "Iniciando Pêndulo de Furuta");
    
    esp_err_t ret = encoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Falha ao inicializar encoder do pêndulo!");
        return;
    }
    
    ret = hw040_encoder_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Falha ao inicializar encoder do braço!");
        return;
    }
    
    ret = motor_init();
    if (ret != ESP_OK) {
        ESP_LOGE("MAIN", "Falha ao inicializar motor!");
        return;
    }
    
    ESP_LOGI("MAIN", "Aguardando 3 segundos para calibração...");
    vTaskDelay(pdMS_TO_TICKS(3000));
    
    encoder_reset_position();
    hw040_encoder_reset_position();
    
    ESP_LOGI("MAIN", "Sistema calibrado. Iniciando controle.");
    
    xTaskCreate(furuta_control_task, "furuta_control", 8192, NULL, 5, NULL);
    
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
