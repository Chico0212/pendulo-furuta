#include "furuta_physics.h"
#include "esp_timer.h"

static const char *TAG = "FURUTA_PHYSICS";

#define DEFAULT_ARM_LENGTH_M 0.15f
#define DEFAULT_PENDULUM_LENGTH_M 0.12f
#define DEFAULT_PENDULUM_MASS_KG 0.05f
#define DEFAULT_ARM_MASS_KG 0.08f
#define DEFAULT_GRAVITY_MS2 9.81f
#define DEFAULT_FRICTION_COEFF 0.01f

esp_err_t furuta_physics_init(furuta_physics_t *physics) {
    if (physics == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    physics->params.arm_length_m = DEFAULT_ARM_LENGTH_M;
    physics->params.pendulum_length_m = DEFAULT_PENDULUM_LENGTH_M;
    physics->params.pendulum_mass_kg = DEFAULT_PENDULUM_MASS_KG;
    physics->params.arm_mass_kg = DEFAULT_ARM_MASS_KG;
    physics->params.gravity_ms2 = DEFAULT_GRAVITY_MS2;
    physics->params.friction_coeff = DEFAULT_FRICTION_COEFF;
    
    physics->params.arm_inertia_kg_m2 = (1.0f/3.0f) * physics->params.arm_mass_kg * 
                                       physics->params.arm_length_m * physics->params.arm_length_m;
    
    physics->params.pendulum_inertia_kg_m2 = (1.0f/3.0f) * physics->params.pendulum_mass_kg * 
                                            physics->params.pendulum_length_m * physics->params.pendulum_length_m;
    
    furuta_physics_reset_history(physics);
    
    ESP_LOGI(TAG, "Physics initialized with parameters:");
    ESP_LOGI(TAG, "  Arm length: %.3f m", physics->params.arm_length_m);
    ESP_LOGI(TAG, "  Pendulum length: %.3f m", physics->params.pendulum_length_m);
    ESP_LOGI(TAG, "  Pendulum mass: %.3f kg", physics->params.pendulum_mass_kg);
    ESP_LOGI(TAG, "  Arm mass: %.3f kg", physics->params.arm_mass_kg);
    
    return ESP_OK;
}

void furuta_physics_set_parameters(furuta_physics_t *physics, furuta_parameters_t *params) {
    if (physics == NULL || params == NULL) {
        return;
    }
    
    physics->params = *params;
    
    physics->params.arm_inertia_kg_m2 = (1.0f/3.0f) * physics->params.arm_mass_kg * 
                                       physics->params.arm_length_m * physics->params.arm_length_m;
    
    physics->params.pendulum_inertia_kg_m2 = (1.0f/3.0f) * physics->params.pendulum_mass_kg * 
                                            physics->params.pendulum_length_m * physics->params.pendulum_length_m;
    
    ESP_LOGI(TAG, "Physics parameters updated");
}

void furuta_physics_update_state(furuta_physics_t *physics, 
                                float arm_angle_rad, 
                                float pendulum_angle_rad, 
                                float timestamp_s) {
    if (physics == NULL) {
        return;
    }
    
    physics->history_index = (physics->history_index + 1) % FURUTA_PHYSICS_MAX_HISTORY;
    if (physics->history_count < FURUTA_PHYSICS_MAX_HISTORY) {
        physics->history_count++;
    }
    
    furuta_physics_state_t *current = &physics->history[physics->history_index];
    current->timestamp_s = timestamp_s;
    current->arm_angle_rad = furuta_physics_normalize_angle(arm_angle_rad);
    current->pendulum_angle_rad = furuta_physics_normalize_angle(pendulum_angle_rad);
    
    current->arm_velocity_rad_s = furuta_physics_estimate_arm_velocity(physics);
    current->pendulum_velocity_rad_s = furuta_physics_estimate_pendulum_velocity(physics);
    
    physics->last_update_time_s = timestamp_s;
}

furuta_physics_state_t furuta_physics_get_current_state(furuta_physics_t *physics) {
    furuta_physics_state_t empty_state = {0};
    
    if (physics == NULL || physics->history_count == 0) {
        return empty_state;
    }
    
    return physics->history[physics->history_index];
}

float furuta_physics_estimate_pendulum_velocity(furuta_physics_t *physics) {
    if (physics == NULL || physics->history_count < 2) {
        return 0.0f;
    }
    
    int current_idx = physics->history_index;
    int prev_idx = (current_idx - 1 + FURUTA_PHYSICS_MAX_HISTORY) % FURUTA_PHYSICS_MAX_HISTORY;
    
    furuta_physics_state_t *current = &physics->history[current_idx];
    furuta_physics_state_t *previous = &physics->history[prev_idx];
    
    float dt = current->timestamp_s - previous->timestamp_s;
    if (dt <= 0.0f) {
        return 0.0f;
    }
    
    float angle_diff = current->pendulum_angle_rad - previous->pendulum_angle_rad;
    
    if (angle_diff > M_PI) {
        angle_diff -= 2.0f * M_PI;
    } else if (angle_diff < -M_PI) {
        angle_diff += 2.0f * M_PI;
    }
    
    return angle_diff / dt;
}

float furuta_physics_estimate_arm_velocity(furuta_physics_t *physics) {
    if (physics == NULL || physics->history_count < 2) {
        return 0.0f;
    }
    
    int current_idx = physics->history_index;
    int prev_idx = (current_idx - 1 + FURUTA_PHYSICS_MAX_HISTORY) % FURUTA_PHYSICS_MAX_HISTORY;
    
    furuta_physics_state_t *current = &physics->history[current_idx];
    furuta_physics_state_t *previous = &physics->history[prev_idx];
    
    float dt = current->timestamp_s - previous->timestamp_s;
    if (dt <= 0.0f) {
        return 0.0f;
    }
    
    float angle_diff = current->arm_angle_rad - previous->arm_angle_rad;
    
    if (angle_diff > M_PI) {
        angle_diff -= 2.0f * M_PI;
    } else if (angle_diff < -M_PI) {
        angle_diff += 2.0f * M_PI;
    }
    
    return angle_diff / dt;
}

float furuta_physics_calculate_energy(furuta_physics_t *physics) {
    if (physics == NULL || physics->history_count == 0) {
        return 0.0f;
    }
    
    furuta_physics_state_t current = furuta_physics_get_current_state(physics);
    
    float pendulum_kinetic = 0.5f * physics->params.pendulum_inertia_kg_m2 * 
                            current.pendulum_velocity_rad_s * current.pendulum_velocity_rad_s;
    
    float arm_kinetic = 0.5f * physics->params.arm_inertia_kg_m2 * 
                       current.arm_velocity_rad_s * current.arm_velocity_rad_s;
    
    float potential = physics->params.pendulum_mass_kg * physics->params.gravity_ms2 * 
                     physics->params.pendulum_length_m * (1.0f + cosf(current.pendulum_angle_rad));
    
    return pendulum_kinetic + arm_kinetic + potential;
}

float furuta_physics_calculate_pendulum_acceleration(furuta_physics_t *physics, float control_torque) {
    if (physics == NULL) {
        return 0.0f;
    }
    
    furuta_physics_state_t current = furuta_physics_get_current_state(physics);
    
    float g = physics->params.gravity_ms2;
    float l = physics->params.pendulum_length_m;
    float m = physics->params.pendulum_mass_kg;
    
    float pendulum_acceleration = -(g / l) * sinf(current.pendulum_angle_rad) + 
                                 control_torque / (m * l * l);
    
    return pendulum_acceleration;
}

float furuta_physics_normalize_angle(float angle_rad) {
    while (angle_rad > M_PI) {
        angle_rad -= 2.0f * M_PI;
    }
    while (angle_rad < -M_PI) {
        angle_rad += 2.0f * M_PI;
    }
    return angle_rad;
}

bool furuta_physics_is_pendulum_up(float pendulum_angle_rad, float threshold_rad) {
    float angle_from_up = furuta_physics_get_angle_from_up(pendulum_angle_rad);
    return angle_from_up < threshold_rad;
}

float furuta_physics_get_angle_from_up(float pendulum_angle_rad) {
    float normalized = furuta_physics_normalize_angle(pendulum_angle_rad);
    float angle_from_up = fabs(normalized);
    
    if (angle_from_up > M_PI) {
        angle_from_up = 2.0f * M_PI - angle_from_up;
    }
    
    return angle_from_up;
}

void furuta_physics_reset_history(furuta_physics_t *physics) {
    if (physics == NULL) {
        return;
    }
    
    physics->history_index = 0;
    physics->history_count = 0;
    physics->last_update_time_s = 0.0f;
    
    for (int i = 0; i < FURUTA_PHYSICS_MAX_HISTORY; i++) {
        physics->history[i].timestamp_s = 0.0f;
        physics->history[i].arm_angle_rad = 0.0f;
        physics->history[i].pendulum_angle_rad = 0.0f;
        physics->history[i].arm_velocity_rad_s = 0.0f;
        physics->history[i].pendulum_velocity_rad_s = 0.0f;
    }
    
    ESP_LOGI(TAG, "Physics history reset");
}