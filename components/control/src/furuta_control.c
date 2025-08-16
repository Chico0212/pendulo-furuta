#include "furuta_control.h"

static const char *TAG = "FURUTA_CONTROL";

#define FURUTA_ARM_LENGTH_M 0.15f
#define FURUTA_PENDULUM_LENGTH_M 0.12f
#define FURUTA_PENDULUM_MASS_KG 0.05f
#define FURUTA_GRAVITY_MS2 9.81f

#define FURUTA_BALANCE_THRESHOLD_RAD (10.0f * M_PI / 180.0f)
#define FURUTA_MAX_ARM_VELOCITY_RAD_S (4.0f * M_PI)
#define FURUTA_MAX_PENDULUM_ANGLE_RAD (60.0f * M_PI / 180.0f)

esp_err_t furuta_control_init(furuta_controller_t *controller) {
    if (controller == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    furuta_control_set_pid_gains(&controller->arm_position_pid, 2.0f, 0.1f, 0.5f);
    furuta_control_set_pid_gains(&controller->pendulum_balance_pid, 15.0f, 0.5f, 1.2f);
    
    controller->swing_up_energy_target = FURUTA_PENDULUM_MASS_KG * FURUTA_GRAVITY_MS2 * FURUTA_PENDULUM_LENGTH_M;
    controller->balance_threshold_rad = FURUTA_BALANCE_THRESHOLD_RAD;
    controller->safety_arm_velocity_limit = FURUTA_MAX_ARM_VELOCITY_RAD_S;
    controller->safety_pendulum_angle_limit = FURUTA_MAX_PENDULUM_ANGLE_RAD;
    
    controller->arm_position_pid.integral_limit = 50.0f;
    controller->arm_position_pid.output_limit = 100.0f;
    controller->pendulum_balance_pid.integral_limit = 20.0f;
    controller->pendulum_balance_pid.output_limit = 100.0f;
    
    furuta_control_reset(controller);
    
    ESP_LOGI(TAG, "Furuta pendulum controller initialized");
    ESP_LOGI(TAG, "Arm PID: Kp=%.2f, Ki=%.2f, Kd=%.2f", 
             controller->arm_position_pid.kp,
             controller->arm_position_pid.ki,
             controller->arm_position_pid.kd);
    ESP_LOGI(TAG, "Pendulum PID: Kp=%.2f, Ki=%.2f, Kd=%.2f",
             controller->pendulum_balance_pid.kp,
             controller->pendulum_balance_pid.ki,
             controller->pendulum_balance_pid.kd);
    
    return ESP_OK;
}

void furuta_control_reset(furuta_controller_t *controller) {
    if (controller == NULL) return;
    
    pid_reset(&controller->arm_position_pid);
    pid_reset(&controller->pendulum_balance_pid);
    
    ESP_LOGI(TAG, "Controller reset");
}

float furuta_control_update(furuta_controller_t *controller, 
                           furuta_state_t *state, 
                           furuta_control_mode_t mode) {
    if (controller == NULL || state == NULL) {
        return 0.0f;
    }
    
    if (!furuta_control_safety_check(state, controller)) {
        ESP_LOGW(TAG, "Safety check failed - emergency stop");
        return 0.0f;
    }
    
    switch (mode) {
        case FURUTA_MODE_SWING_UP:
            return furuta_swing_up_energy_control(state, controller->swing_up_energy_target);
            
        case FURUTA_MODE_BALANCE:
            return furuta_balance_control(controller, state);
            
        case FURUTA_MODE_MANUAL:
            return 0.0f;
            
        case FURUTA_MODE_IDLE:
        case FURUTA_MODE_EMERGENCY_STOP:
        default:
            return 0.0f;
    }
}

furuta_control_mode_t furuta_control_determine_mode(furuta_state_t *state, 
                                                   furuta_control_mode_t current_mode) {
    if (state == NULL) {
        return FURUTA_MODE_EMERGENCY_STOP;
    }
    
    float pendulum_angle_from_up = fabs(state->pendulum_angle_rad);
    if (pendulum_angle_from_up > M_PI) {
        pendulum_angle_from_up = 2.0f * M_PI - pendulum_angle_from_up;
    }
    
    switch (current_mode) {
        case FURUTA_MODE_IDLE:
            if (pendulum_angle_from_up > FURUTA_BALANCE_THRESHOLD_RAD) {
                ESP_LOGD(TAG, "Switching to SWING_UP mode");
                return FURUTA_MODE_SWING_UP;
            }
            break;
            
        case FURUTA_MODE_SWING_UP:
            if (pendulum_angle_from_up < FURUTA_BALANCE_THRESHOLD_RAD) {
                ESP_LOGD(TAG, "Switching to BALANCE mode");
                return FURUTA_MODE_BALANCE;
            }
            break;
            
        case FURUTA_MODE_BALANCE:
            if (pendulum_angle_from_up > FURUTA_BALANCE_THRESHOLD_RAD * 3.0f) {
                ESP_LOGD(TAG, "Lost balance - switching to SWING_UP mode");
                return FURUTA_MODE_SWING_UP;
            }
            break;
            
        default:
            break;
    }
    
    return current_mode;
}

bool furuta_control_safety_check(furuta_state_t *state, furuta_controller_t *controller) {
    if (state == NULL || controller == NULL) {
        return false;
    }
    
    if (fabs(state->arm_velocity_rad_s) > controller->safety_arm_velocity_limit) {
        ESP_LOGW(TAG, "Arm velocity %.2f exceeds limit %.2f", 
                 state->arm_velocity_rad_s, controller->safety_arm_velocity_limit);
        return false;
    }
    
    float pendulum_angle_from_up = fabs(state->pendulum_angle_rad);
    if (pendulum_angle_from_up > M_PI) {
        pendulum_angle_from_up = 2.0f * M_PI - pendulum_angle_from_up;
    }
    
    float pendulum_angle_from_down = M_PI - pendulum_angle_from_up;
    if (pendulum_angle_from_down > controller->safety_pendulum_angle_limit) {
        ESP_LOGW(TAG, "Pendulum angle %.2f° from vertical exceeds safety limit %.2f°", 
                 pendulum_angle_from_down * 180.0f / M_PI,
                 controller->safety_pendulum_angle_limit * 180.0f / M_PI);
        return false;
    }
    
    return true;
}

void furuta_control_set_pid_gains(pid_controller_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

float pid_update(pid_controller_t *pid, float error, float dt) {
    if (pid == NULL || dt <= 0.0f) {
        return 0.0f;
    }
    
    pid->integral += error * dt;
    
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    
    float derivative = (error - pid->previous_error) / dt;
    pid->previous_error = error;
    
    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    
    if (output > pid->output_limit) {
        output = pid->output_limit;
    } else if (output < -pid->output_limit) {
        output = -pid->output_limit;
    }
    
    return output;
}

void pid_reset(pid_controller_t *pid) {
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->previous_error = 0.0f;
}

float furuta_swing_up_energy_control(furuta_state_t *state, float target_energy) {
    if (state == NULL) {
        return 0.0f;
    }
    
    float kinetic_energy = 0.5f * FURUTA_PENDULUM_MASS_KG * 
                          FURUTA_PENDULUM_LENGTH_M * FURUTA_PENDULUM_LENGTH_M * 
                          state->pendulum_velocity_rad_s * state->pendulum_velocity_rad_s;
    
    float potential_energy = FURUTA_PENDULUM_MASS_KG * FURUTA_GRAVITY_MS2 * 
                            FURUTA_PENDULUM_LENGTH_M * (1.0f + cosf(state->pendulum_angle_rad));
    
    float total_energy = kinetic_energy + potential_energy;
    float energy_error = target_energy - total_energy;
    
    float control_output = 0.3f * energy_error * sinf(state->pendulum_angle_rad) * 
                          copysignf(1.0f, state->pendulum_velocity_rad_s);
    
    return fmaxf(-50.0f, fminf(50.0f, control_output));
}

float furuta_balance_control(furuta_controller_t *controller, furuta_state_t *state) {
    if (controller == NULL || state == NULL) {
        return 0.0f;
    }
    
    float pendulum_error = -state->pendulum_angle_rad;
    float pendulum_output = pid_update(&controller->pendulum_balance_pid, 
                                      pendulum_error, 
                                      FURUTA_CONTROL_PERIOD_S);
    
    float arm_target = pendulum_output * 0.1f;
    float arm_error = arm_target - state->arm_angle_rad;
    float arm_output = pid_update(&controller->arm_position_pid, 
                                 arm_error, 
                                 FURUTA_CONTROL_PERIOD_S);
    
    return arm_output;
}