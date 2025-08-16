// // // =============================================================================
// // // FURUTA PENDULUM CONTROL SYSTEM - MAIN APPLICATION
// // // =============================================================================
// // // This file implements the main control loop for a Furuta pendulum system.
// // // A Furuta pendulum is an inverted pendulum mounted on a rotating arm.
// // // The system uses real-time control algorithms to swing up and balance the
// // // pendulum in the upright position.
// // //
// // // Hardware components:
// // // - ESP32 microcontroller
// // // - Stepper motor with Monster Moto Mini driver (arm rotation)
// // // - A38S6-400-2-24 rotary encoder (pendulum angle, 400 PPR)
// // // - HW-040 rotary encoder (arm angle, 20 PPR)
// // // - 12V power supply for A38 encoder, 3.3V/5V for HW-040
// // // =============================================================================

// // #include "motor.h"           // Stepper motor control with PWM and direction
// // #include "encoder.h"         // A38 pendulum encoder (high resolution)
// // #include "hw040_encoder.h"   // HW-040 arm encoder (low resolution)
// // #include "furuta_control.h"  // PID controllers and control algorithms
// // #include "furuta_physics.h"  // Physics simulation and state estimation
// // #include "furuta_sensors.h"  // Sensor fusion and data validation
// // #include "freertos/task.h"   // FreeRTOS task management
// // #include "esp_timer.h"       // High-precision timing

// // // Logging tag for ESP-IDF logging system
// // static const char *TAG = "FURUTA_PENDULUM";

// // // Control loop timing configuration
// // // 100 Hz provides good balance between responsiveness and CPU usage
// // #define CONTROL_LOOP_FREQUENCY_HZ 100
// // #define CONTROL_LOOP_PERIOD_MS (1000 / CONTROL_LOOP_FREQUENCY_HZ)

// // // Global system state objects
// // // These maintain the state of the control system components
// // static furuta_controller_t g_controller;  // PID controllers and control logic
// // static furuta_physics_t g_physics;        // Physics simulation and state estimation
// // static furuta_sensors_t g_sensors;        // Sensor data processing and validation
// // static furuta_control_mode_t g_current_mode = FURUTA_MODE_IDLE; // Current control mode

// // // Function prototypes
// // static void furuta_pendulum_control_task(void *pvParameters);
// // static esp_err_t furuta_pendulum_init(void);
// // static void furuta_system_calibration(void);

// // void app_main(void)
// // {
// //     ESP_LOGI(TAG, "=== Furuta Pendulum Control System ===");
// //     ESP_LOGI(TAG, "Version: 1.0");
// //     ESP_LOGI(TAG, "Control Frequency: %d Hz", CONTROL_LOOP_FREQUENCY_HZ);

// //     esp_err_t ret = furuta_pendulum_init();
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "System initialization failed: %s", esp_err_to_name(ret));
// //         return;
// //     }

// //     ESP_LOGI(TAG, "Starting calibration sequence...");
// //     furuta_system_calibration();

// //     ESP_LOGI(TAG, "System ready - starting control loop");
// //     g_current_mode = FURUTA_MODE_SWING_UP;

// //     xTaskCreate(furuta_pendulum_control_task, "furuta_control", 8192, NULL, 5, NULL);
// // }

// // static esp_err_t furuta_pendulum_init(void)
// // {
// //     esp_err_t ret;

// //     ESP_LOGI(TAG, "Initializing hardware components...");

// //     ret = motor_init();
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "Motor initialization failed: %s", esp_err_to_name(ret));
// //         return ret;
// //     }

// //     ret = encoder_init();
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "Pendulum encoder (A38) initialization failed: %s", esp_err_to_name(ret));
// //         return ret;
// //     }

// //     ret = hw040_encoder_init();
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "Arm encoder (HW-040) initialization failed: %s", esp_err_to_name(ret));
// //         return ret;
// //     }

// //     ESP_LOGI(TAG, "Initializing control components...");

// //     ret = furuta_control_init(&g_controller);
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "Controller initialization failed: %s", esp_err_to_name(ret));
// //         return ret;
// //     }

// //     ret = furuta_physics_init(&g_physics);
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "Physics initialization failed: %s", esp_err_to_name(ret));
// //         return ret;
// //     }

// //     ret = furuta_sensors_init(&g_sensors);
// //     if (ret != ESP_OK) {
// //         ESP_LOGE(TAG, "Sensors initialization failed: %s", esp_err_to_name(ret));
// //         return ret;
// //     }

// //     encoder_reset_position();
// //     motor_stop();

// //     ESP_LOGI(TAG, "All components initialized successfully");
// //     return ESP_OK;
// // }

// // static void furuta_system_calibration(void)
// // {
// //     ESP_LOGI(TAG, "=== CALIBRATION PHASE ===");
// //     ESP_LOGI(TAG, "Please ensure pendulum is hanging down and system is still");

// //     vTaskDelay(pdMS_TO_TICKS(2000));

// //     furuta_sensors_calibrate_offsets(&g_sensors);

// //     ESP_LOGI(TAG, "Calibration complete");
// // }

// // static void furuta_pendulum_control_task(void *pvParameters)
// // {
// //     TickType_t xLastWakeTime = xTaskGetTickCount();
// //     uint32_t loop_counter = 0;

// //     ESP_LOGI(TAG, "Control task started");

// //     while (1) {
// //         float timestamp_s = esp_timer_get_time() / 1000000.0f;

// //         furuta_sensor_data_t sensor_data = furuta_sensors_read_all(&g_sensors);

// //         if (!furuta_sensors_is_data_valid(&sensor_data)) {
// //             ESP_LOGW(TAG, "Invalid sensor data - emergency stop");
// //             motor_emergency_stop();
// //             g_current_mode = FURUTA_MODE_EMERGENCY_STOP;
// //             vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
// //             continue;
// //         }

// //         furuta_physics_update_state(&g_physics,
// //                                    sensor_data.arm_angle_rad,
// //                                    sensor_data.pendulum_angle_rad,
// //                                    timestamp_s);

// //         furuta_physics_state_t physics_state = furuta_physics_get_current_state(&g_physics);

// //         furuta_state_t control_state = {
// //             .arm_angle_rad = physics_state.arm_angle_rad,
// //             .arm_velocity_rad_s = physics_state.arm_velocity_rad_s,
// //             .pendulum_angle_rad = physics_state.pendulum_angle_rad,
// //             .pendulum_velocity_rad_s = physics_state.pendulum_velocity_rad_s
// //         };

// //         g_current_mode = furuta_control_determine_mode(&control_state, g_current_mode);

// //         float control_output = furuta_control_update(&g_controller, &control_state, g_current_mode);

// //         if (g_current_mode != FURUTA_MODE_EMERGENCY_STOP && g_current_mode != FURUTA_MODE_IDLE) {
// //             motor_set_velocity_control(control_output);
// //         } else {
// //             motor_stop();
// //         }

// //         // Print status every 5 seconds (500 loops at 100Hz)
// //         if (loop_counter % 500 == 0) {
// //             const char* mode_str = (g_current_mode == FURUTA_MODE_SWING_UP) ? "SWING_UP" :
// //                                   (g_current_mode == FURUTA_MODE_BALANCE) ? "BALANCE" :
// //                                   (g_current_mode == FURUTA_MODE_IDLE) ? "IDLE" : "E_STOP";

// //             ESP_LOGI(TAG, "Mode: %s | Arm: %.1f° %.1f°/s | Pendulum: %.1f° %.1f°/s | Out: %.1f",
// //                      mode_str,
// //                      control_state.arm_angle_rad * 180.0f / M_PI,
// //                      control_state.arm_velocity_rad_s * 180.0f / M_PI,
// //                      control_state.pendulum_angle_rad * 180.0f / M_PI,
// //                      control_state.pendulum_velocity_rad_s * 180.0f / M_PI,
// //                      control_output);
// //         }

// //         loop_counter++;
// //         vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(CONTROL_LOOP_PERIOD_MS));
// //     }
// // }

#include "motor.h"         // Stepper motor control
#include "encoder.h"       // A38 pendulum encoder
#include "hw040_encoder.h" // HW-040 arm encoder
#include "freertos/task.h"
#include "esp_log.h"

// static const char *TAG = "ENCODER_TEST";

// void encoder_test_task(void *pvParameters)
// {
//     // Store previous encoder positions to detect movement
//     static int64_t prev_pendulum_pos = 0;
//     static int64_t prev_arm_pos = 0;
    
//     // Get initial positions
//     prev_pendulum_pos = encoder_get_position();
//     prev_arm_pos = hw040_encoder_get_position();
    
//     while(1) {
//         // Read current encoder positions
//         int64_t pendulum_pos = encoder_get_position();
//         float pendulum_angle_deg = encoder_get_pendulum_angle_degrees();
        
//         int64_t arm_pos = hw040_encoder_get_position();
//         float arm_angle_deg = hw040_get_arm_angle_degrees();
        
//         // Calculate position changes
//         int64_t pendulum_delta = pendulum_pos - prev_pendulum_pos;
//         int64_t arm_delta = arm_pos - prev_arm_pos;
        
//         // Print encoder values
//         ESP_LOGI(TAG, "Pendulum: pos=%lld (Δ%lld), angle=%.1f°  |  Arm: pos=%lld (Δ%lld), angle=%.1f°", 
//                  pendulum_pos, pendulum_delta, pendulum_angle_deg, arm_pos, arm_delta, arm_angle_deg);
        
//         // Control motor based on pendulum encoder movement
//         if (pendulum_delta != 0) {
//             // Determine direction and number of steps based on encoder movement
//             int steps_to_move = abs((int)pendulum_delta) / 4; // Scale down encoder counts to motor steps
//             if (steps_to_move < 1) steps_to_move = 1; // Minimum 1 step
//             if (steps_to_move > 10) steps_to_move = 10; // Maximum 10 steps per cycle
            
//             ESP_LOGI(TAG, "Pendulum moved %lld counts -> Motor steps: %d %s", 
//                      pendulum_delta, steps_to_move, 
//                      (pendulum_delta > 0) ? "clockwise" : "counter-clockwise");
            
//             // Set motor speed (lower delay = faster)
//             motor_set_step_delay(20); // Fast response
            
//             // Move motor in appropriate direction
//             for (int i = 0; i < steps_to_move; i++) {
//                 if (pendulum_delta > 0) {
//                     motor_step_clockwise();
//                 } else {
//                     motor_step_counter_clockwise();
//                 }
//             }
            
//             // Stop motor after movement
//             motor_stop();
//         }
        
//         // Update previous positions
//         prev_pendulum_pos = pendulum_pos;
//         prev_arm_pos = arm_pos;
        
//         vTaskDelay(pdMS_TO_TICKS(50)); // Faster response time (20Hz)
//     }
// }

// void app_main()
// {
//     ESP_LOGI(TAG, "Starting encoder and motor test");
    
//     esp_err_t err = motor_init();
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(err));
//         return;
//     }
    
//     err = encoder_init();
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Pendulum encoder init failed: %s", esp_err_to_name(err));
//         return;
//     }
    
//     err = hw040_encoder_init();
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "Arm encoder init failed: %s", esp_err_to_name(err));
//         return;
//     }
    
//     ESP_LOGI(TAG, "All components initialized successfully");
    
//     // Reset encoder positions to zero
//     encoder_reset_position();
//     hw040_encoder_reset_position();
    
//     // Create task to read encoders and move motor
//     xTaskCreate(encoder_test_task, "encoder_test", 4096, NULL, 5, NULL);
// }


static const char *TAG = "ENCODER_MOTOR_TEST";

void encoder_motor_task(void *pvParameters)
{
    // Store previous encoder position to detect movement
    int64_t prev_pendulum_pos = encoder_get_position();
    
    ESP_LOGI(TAG, "Encoder-motor control started. Move the pendulum encoder to control motor.");
    ESP_LOGI(TAG, "Initial pendulum position: %lld", prev_pendulum_pos);
    
    while(1) {
        // Read current encoder position
        int64_t current_pendulum_pos = encoder_get_position();
        float pendulum_angle_deg = encoder_get_pendulum_angle_degrees();
        
        // Calculate position change
        int64_t position_delta = current_pendulum_pos - prev_pendulum_pos;
        
        // Only print and move motor if there's movement
        if (position_delta != 0) {
            ESP_LOGI(TAG, "Encoder moved: %lld counts (angle: %.1f°)", position_delta, pendulum_angle_deg);
            
            // Calculate motor steps based on encoder movement
            int motor_steps = abs((int)position_delta) / 2; // Scale down movement
            if (motor_steps < 1) motor_steps = 1; // Minimum 1 step
            if (motor_steps > 20) motor_steps = 20; // Maximum 20 steps
            
            // Move motor in same direction as encoder
            ESP_LOGI(TAG, "Moving motor %d steps %s", motor_steps, 
                     (position_delta > 0) ? "clockwise" : "counter-clockwise");
            
            for (int i = 0; i < motor_steps; i++) {
                if (position_delta > 0) {
                    motor_step_clockwise();
                } else {
                    motor_step_counter_clockwise();
                }
            }
            
            // Update previous position
            prev_pendulum_pos = current_pendulum_pos;
        }
        
        // Check every 50ms for responsive control
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main() {
    ESP_LOGI(TAG, "Encoder-Motor Test - Starting...");
    
    // Initialize motor
    esp_err_t err = motor_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Motor init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Motor initialized successfully");
    
    // Initialize pendulum encoder
    err = encoder_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Encoder init failed: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "Pendulum encoder initialized successfully");
    
    // Reset encoder position to zero
    encoder_reset_position();
    
    // Set motor speed (fast response)
    motor_set_step_delay(15); // 15ms between steps
    
    // Create task to monitor encoder and control motor
    xTaskCreate(encoder_motor_task, "encoder_motor", 4096, NULL, 5, NULL);
}
