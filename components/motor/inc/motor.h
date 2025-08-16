#ifndef MOTOR_H
#define MOTOR_H

#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <math.h>
#include <stdbool.h>

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

// Open-loop velocity control (no encoder feedback)
void motor_set_velocity_control(float velocity_dps);

// Open-loop angle control (time-based, no encoder feedback)
void motor_move_to_angle(float target_angle_deg);

// Set continuous rotation velocity (for arm control)
void motor_set_arm_velocity(float velocity_dps);

bool motor_is_moving(void);

int motor_get_current_delay(void);

int motor_get_current_duty(void);

float motor_calculate_rpm_from_delay(int delay_ms);

void motor_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_H