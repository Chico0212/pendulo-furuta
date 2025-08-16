#ifndef FURUTA_PHYSICS_H
#define FURUTA_PHYSICS_H

#include <math.h>
#include <stdbool.h>
#include "esp_log.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define FURUTA_PHYSICS_MAX_HISTORY 10

typedef struct {
    float arm_length_m;
    float pendulum_length_m;
    float pendulum_mass_kg;
    float arm_mass_kg;
    float arm_inertia_kg_m2;
    float pendulum_inertia_kg_m2;
    float gravity_ms2;
    float friction_coeff;
} furuta_parameters_t;

typedef struct {
    float timestamp_s;
    float arm_angle_rad;
    float pendulum_angle_rad;
    float arm_velocity_rad_s;
    float pendulum_velocity_rad_s;
} furuta_physics_state_t;

typedef struct {
    furuta_parameters_t params;
    furuta_physics_state_t history[FURUTA_PHYSICS_MAX_HISTORY];
    int history_index;
    int history_count;
    float last_update_time_s;
} furuta_physics_t;

esp_err_t furuta_physics_init(furuta_physics_t *physics);

void furuta_physics_set_parameters(furuta_physics_t *physics, furuta_parameters_t *params);

void furuta_physics_update_state(furuta_physics_t *physics, 
                                float arm_angle_rad, 
                                float pendulum_angle_rad, 
                                float timestamp_s);

furuta_physics_state_t furuta_physics_get_current_state(furuta_physics_t *physics);

float furuta_physics_estimate_pendulum_velocity(furuta_physics_t *physics);

float furuta_physics_estimate_arm_velocity(furuta_physics_t *physics);

float furuta_physics_calculate_energy(furuta_physics_t *physics);

float furuta_physics_calculate_pendulum_acceleration(furuta_physics_t *physics, float control_torque);

float furuta_physics_normalize_angle(float angle_rad);

bool furuta_physics_is_pendulum_up(float pendulum_angle_rad, float threshold_rad);

float furuta_physics_get_angle_from_up(float pendulum_angle_rad);

void furuta_physics_reset_history(furuta_physics_t *physics);

#ifdef __cplusplus
}
#endif

#endif // FURUTA_PHYSICS_H