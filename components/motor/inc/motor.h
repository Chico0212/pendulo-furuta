#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MOTOR_A1_PIN GPIO_NUM_19
#define MOTOR_B1_PIN GPIO_NUM_21
#define MOTOR_A2_PIN GPIO_NUM_22
#define MOTOR_B2_PIN GPIO_NUM_23

#define MOTOR_PWM_PIN_1 GPIO_NUM_26
#define MOTOR_PWM_PIN_2 GPIO_NUM_27

#define MOTOR_LEDC_TIMER LEDC_TIMER_0
#define MOTOR_LEDC_MODE LEDC_HIGH_SPEED_MODE
#define MOTOR_LEDC_CHANNEL_1 LEDC_CHANNEL_0
#define MOTOR_LEDC_CHANNEL_2 LEDC_CHANNEL_1
#define MOTOR_LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define MOTOR_LEDC_FREQUENCY 10000

#define MOTOR_DEFAULT_STEP_DELAY_MS 17
#define MOTOR_DEFAULT_DUTY_CYCLE 128

typedef enum {
    MOTOR_DIRECTION_CLOCKWISE,
    MOTOR_DIRECTION_COUNTER_CLOCKWISE
} motor_direction_t;

esp_err_t motor_init(void);

void motor_set_speed_rpm(float rpm);

void motor_set_step_delay(int delay_ms);

void motor_set_torque(int duty_cycle);

void motor_set_pwm_speed(int duty_cycle);

void motor_step_clockwise(void);

void motor_step_counter_clockwise(void);

void motor_stop(void);

int motor_get_current_delay(void);

int motor_get_current_duty(void);

float motor_calculate_rpm_from_delay(int delay_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H