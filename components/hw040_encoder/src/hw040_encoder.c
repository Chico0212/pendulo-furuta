#include "hw040_encoder.h"

static const char *TAG = "HW040_ENCODER";
static volatile int64_t hw040_position = 0;
static volatile hw040_direction_t hw040_direction = HW040_DIRECTION_UNKNOWN;
static volatile uint32_t last_pulse_time_us = 0;
static volatile float current_speed_rpm = 0.0f;
static volatile uint8_t last_channel_a = 0;
static volatile uint8_t last_channel_b = 0;

static SemaphoreHandle_t hw040_mutex = NULL;
static esp_timer_handle_t speed_calc_timer = NULL;

static void IRAM_ATTR hw040_channel_a_isr(void *arg);
static void IRAM_ATTR hw040_channel_b_isr(void *arg);
static void hw040_update_position_and_direction(uint8_t channel_a, uint8_t channel_b);
static void hw040_speed_calculation_timer_callback(void *arg);
static esp_err_t hw040_setup_gpio(void);
static esp_err_t hw040_setup_speed_timer(void);

esp_err_t hw040_encoder_init(void) {
    esp_err_t ret;
    
    hw040_mutex = xSemaphoreCreateMutex();
    if (hw040_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create HW-040 mutex");
        return ESP_ERR_NO_MEM;
    }
    
    ret = hw040_setup_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup GPIO: %s", esp_err_to_name(ret));
        vSemaphoreDelete(hw040_mutex);
        return ret;
    }
    
    ret = hw040_setup_speed_timer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup speed timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(hw040_mutex);
        return ret;
    }
    
    last_channel_a = gpio_get_level(HW040_CHANNEL_A_PIN);
    last_channel_b = gpio_get_level(HW040_CHANNEL_B_PIN);
    
    hw040_position = 0;
    hw040_direction = HW040_DIRECTION_UNKNOWN;
    current_speed_rpm = 0.0f;
    last_pulse_time_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "HW-040 arm encoder initialized successfully");
    ESP_LOGI(TAG, "Channel A: GPIO %d, Channel B: GPIO %d", HW040_CHANNEL_A_PIN, HW040_CHANNEL_B_PIN);
    ESP_LOGI(TAG, "Resolution: %d PPR, %d counts per revolution", HW040_PPR, HW040_COUNTS_PER_REV);
    ESP_LOGI(TAG, "Initial Channel A: %d, Channel B: %d", last_channel_a, last_channel_b);
    ESP_LOGI(TAG, "Measuring arm angle: 0° = reference position");
    
    return ESP_OK;
}

static esp_err_t hw040_setup_gpio(void) {
    esp_err_t ret;
    
    gpio_config_t gpio_conf_a = {
        .pin_bit_mask = (1ULL << HW040_CHANNEL_A_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    ret = gpio_config(&gpio_conf_a);
    if (ret != ESP_OK) {
        return ret;
    }
    
    gpio_config_t gpio_conf_b = {
        .pin_bit_mask = (1ULL << HW040_CHANNEL_B_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE
    };
    
    ret = gpio_config(&gpio_conf_b);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        return ret;
    }
    
    ret = gpio_isr_handler_add(HW040_CHANNEL_A_PIN, hw040_channel_a_isr, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = gpio_isr_handler_add(HW040_CHANNEL_B_PIN, hw040_channel_b_isr, NULL);
    if (ret != ESP_OK) {
        gpio_isr_handler_remove(HW040_CHANNEL_A_PIN);
        return ret;
    }
    
    return ESP_OK;
}

static esp_err_t hw040_setup_speed_timer(void) {
    esp_timer_create_args_t timer_args = {
        .callback = hw040_speed_calculation_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "hw040_speed_timer"
    };
    
    esp_err_t ret = esp_timer_create(&timer_args, &speed_calc_timer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return esp_timer_start_periodic(speed_calc_timer, HW040_SPEED_CALC_INTERVAL_US);
}

static void IRAM_ATTR hw040_channel_a_isr(void *arg) {
    uint8_t channel_a = gpio_get_level(HW040_CHANNEL_A_PIN);
    uint8_t channel_b = gpio_get_level(HW040_CHANNEL_B_PIN);
    hw040_update_position_and_direction(channel_a, channel_b);
}

static void IRAM_ATTR hw040_channel_b_isr(void *arg) {
    uint8_t channel_a = gpio_get_level(HW040_CHANNEL_A_PIN);
    uint8_t channel_b = gpio_get_level(HW040_CHANNEL_B_PIN);
    hw040_update_position_and_direction(channel_a, channel_b);
}

static void hw040_update_position_and_direction(uint8_t channel_a, uint8_t channel_b) {
    static const int8_t encoder_table[4][4] = {
        //  00  01  10  11
        {   0, -1,  1,  0},  // 00
        {   1,  0,  0, -1},  // 01
        {  -1,  0,  0,  1},  // 10
        {   0,  1, -1,  0}   // 11
    };
    
    uint8_t current_state = (channel_a << 1) | channel_b;
    uint8_t last_state = (last_channel_a << 1) | last_channel_b;
    
    int8_t delta = encoder_table[last_state][current_state];
    
    if (delta != 0) {
        hw040_position += delta;
        last_pulse_time_us = esp_timer_get_time();
        
        if (delta > 0) {
            hw040_direction = HW040_DIRECTION_CLOCKWISE;
        } else {
            hw040_direction = HW040_DIRECTION_COUNTER_CLOCKWISE;
        }
    }
    
    last_channel_a = channel_a;
    last_channel_b = channel_b;
}

static void hw040_speed_calculation_timer_callback(void *arg) {
    static uint32_t last_calc_time_us = 0;
    static int64_t last_position = 0;
    
    uint32_t current_time_us = esp_timer_get_time();
    
    if (last_calc_time_us == 0) {
        last_calc_time_us = current_time_us;
        last_position = hw040_position;
        return;
    }
    
    uint32_t time_diff_us = current_time_us - last_calc_time_us;
    int64_t position_diff = hw040_position - last_position;
    
    if (time_diff_us > 0) {
        float revolutions = (float)position_diff / HW040_COUNTS_PER_REV;
        float time_diff_minutes = (float)time_diff_us / 60000000.0f;
        current_speed_rpm = revolutions / time_diff_minutes;
    }
    
    uint32_t time_since_last_pulse = current_time_us - last_pulse_time_us;
    if (time_since_last_pulse > 500000) {  // 500ms timeout
        current_speed_rpm = 0.0f;
        hw040_direction = HW040_DIRECTION_UNKNOWN;
    }
    
    last_calc_time_us = current_time_us;
    last_position = hw040_position;
}

void hw040_encoder_reset_position(void) {
    if (xSemaphoreTake(hw040_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        hw040_position = 0;
        current_speed_rpm = 0.0f;
        hw040_direction = HW040_DIRECTION_UNKNOWN;
        xSemaphoreGive(hw040_mutex);
        ESP_LOGI(TAG, "HW-040 encoder position reset to 0");
    }
}

int64_t hw040_encoder_get_position(void) {
    int64_t position = 0;
    if (xSemaphoreTake(hw040_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        position = hw040_position;
        xSemaphoreGive(hw040_mutex);
    }
    return position;
}

float hw040_encoder_get_speed_rpm(void) {
    float speed = 0.0f;
    if (xSemaphoreTake(hw040_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        speed = current_speed_rpm;
        xSemaphoreGive(hw040_mutex);
    }
    return speed;
}

hw040_direction_t hw040_encoder_get_direction(void) {
    hw040_direction_t direction = HW040_DIRECTION_UNKNOWN;
    if (xSemaphoreTake(hw040_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        direction = hw040_direction;
        xSemaphoreGive(hw040_mutex);
    }
    return direction;
}

hw040_data_t hw040_encoder_get_data(void) {
    hw040_data_t data = {0};
    if (xSemaphoreTake(hw040_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data.position = hw040_position;
        data.speed_rpm = current_speed_rpm;
        data.direction = hw040_direction;
        data.last_pulse_time_us = last_pulse_time_us;
        xSemaphoreGive(hw040_mutex);
    }
    return data;
}

float hw040_position_to_degrees(int64_t position) {
    return (float)position * 360.0f / HW040_COUNTS_PER_REV;
}

float hw040_position_to_radians(int64_t position) {
    return (float)position * 2.0f * M_PI / HW040_COUNTS_PER_REV;
}

int64_t hw040_degrees_to_position(float degrees) {
    return (int64_t)(degrees * HW040_COUNTS_PER_REV / 360.0f);
}

int64_t hw040_radians_to_position(float radians) {
    return (int64_t)(radians * HW040_COUNTS_PER_REV / (2.0f * M_PI));
}

float hw040_get_arm_angle_degrees(void) {
    int64_t position = hw040_encoder_get_position();
    float angle = hw040_position_to_degrees(position);
    
    // Normalize to 0-360° range
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    
    return angle;
}

float hw040_get_arm_angle_radians(void) {
    int64_t position = hw040_encoder_get_position();
    float angle = hw040_position_to_radians(position);
    
    // Normalize to 0-2π range
    while (angle < 0) angle += 2.0f * M_PI;
    while (angle >= 2.0f * M_PI) angle -= 2.0f * M_PI;
    
    return angle;
}