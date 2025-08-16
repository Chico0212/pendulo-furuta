#ifndef FURUTA_CONTROL_H
#define FURUTA_CONTROL_H

#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FURUTA_CONTROL_FREQUENCY_HZ 100.0f
#define FURUTA_CONTROL_PERIOD_S (1.0f / FURUTA_CONTROL_FREQUENCY_HZ)

typedef enum {
    FURUTA_MODE_IDLE,
    FURUTA_MODE_SWING_UP,
    FURUTA_MODE_BALANCE,
    FURUTA_MODE_MANUAL,
    FURUTA_MODE_EMERGENCY_STOP
} furuta_control_mode_t;

typedef struct {
    float arm_angle_rad;
    float arm_velocity_rad_s;
    float pendulum_angle_rad;
    float pendulum_velocity_rad_s;
} furuta_state_t;

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
    float integral_limit;
    float output_limit;
} pid_controller_t;

typedef struct {
    pid_controller_t arm_position_pid;
    pid_controller_t pendulum_balance_pid;
    float swing_up_energy_target;
    float balance_threshold_rad;
    float safety_arm_velocity_limit;
    float safety_pendulum_angle_limit;
} furuta_controller_t;

esp_err_t furuta_control_init(furuta_controller_t *controller);

void furuta_control_reset(furuta_controller_t *controller);

float furuta_control_update(furuta_controller_t *controller, 
                           furuta_state_t *state, 
                           furuta_control_mode_t mode);

furuta_control_mode_t furuta_control_determine_mode(furuta_state_t *state, 
                                                   furuta_control_mode_t current_mode);

bool furuta_control_safety_check(furuta_state_t *state, furuta_controller_t *controller);

void furuta_control_set_pid_gains(pid_controller_t *pid, float kp, float ki, float kd);

float pid_update(pid_controller_t *pid, float error, float dt);

void pid_reset(pid_controller_t *pid);

float furuta_swing_up_energy_control(furuta_state_t *state, float target_energy);

float furuta_balance_control(furuta_controller_t *controller, furuta_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // FURUTA_CONTROL_H