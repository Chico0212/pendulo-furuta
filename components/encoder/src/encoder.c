#include "encoder.h"

static const char *TAG = "ENCODER";
static volatile int64_t encoder_position = 0;
static volatile encoder_direction_t encoder_direction = ENCODER_DIRECTION_UNKNOWN;
static volatile uint32_t last_pulse_time_us = 0;
static volatile float current_speed_rpm = 0.0f;
static volatile uint8_t last_channel_a = 0;
static volatile uint8_t last_channel_b = 0;

static SemaphoreHandle_t encoder_mutex = NULL;
static esp_timer_handle_t speed_calc_timer = NULL;

static void IRAM_ATTR encoder_channel_a_isr(void *arg);
static void IRAM_ATTR encoder_channel_b_isr(void *arg);
static void encoder_update_position_and_direction(uint8_t channel_a, uint8_t channel_b);
static void speed_calculation_timer_callback(void *arg);
static esp_err_t encoder_setup_gpio(void);
static esp_err_t encoder_setup_speed_timer(void);

esp_err_t encoder_init(void) {
    esp_err_t ret;
    
    encoder_mutex = xSemaphoreCreateMutex();
    if (encoder_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create encoder mutex");
        return ESP_ERR_NO_MEM;
    }
    
    ret = encoder_setup_gpio();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup GPIO: %s", esp_err_to_name(ret));
        vSemaphoreDelete(encoder_mutex);
        return ret;
    }
    
    ret = encoder_setup_speed_timer();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to setup speed timer: %s", esp_err_to_name(ret));
        vSemaphoreDelete(encoder_mutex);
        return ret;
    }
    
    last_channel_a = gpio_get_level(ENCODER_CHANNEL_A_PIN);
    last_channel_b = gpio_get_level(ENCODER_CHANNEL_B_PIN);
    
    encoder_position = 0;
    encoder_direction = ENCODER_DIRECTION_UNKNOWN;
    current_speed_rpm = 0.0f;
    last_pulse_time_us = esp_timer_get_time();
    
    ESP_LOGI(TAG, "Encoder initialized successfully");
    ESP_LOGI(TAG, "Channel A: GPIO %d, Channel B: GPIO %d", ENCODER_CHANNEL_A_PIN, ENCODER_CHANNEL_B_PIN);
    ESP_LOGI(TAG, "Resolution: %d PPR, %d counts per revolution", ENCODER_PPR, ENCODER_COUNTS_PER_REV);
    ESP_LOGI(TAG, "Initial Channel A: %d, Channel B: %d", last_channel_a, last_channel_b);
    
    return ESP_OK;
}

static esp_err_t encoder_setup_gpio(void) {
    esp_err_t ret;
    
    gpio_config_t gpio_conf_a = {
        .pin_bit_mask = (1ULL << ENCODER_CHANNEL_A_PIN),
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
        .pin_bit_mask = (1ULL << ENCODER_CHANNEL_B_PIN),
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
    
    ret = gpio_isr_handler_add(ENCODER_CHANNEL_A_PIN, encoder_channel_a_isr, NULL);
    if (ret != ESP_OK) {
        return ret;
    }
    
    ret = gpio_isr_handler_add(ENCODER_CHANNEL_B_PIN, encoder_channel_b_isr, NULL);
    if (ret != ESP_OK) {
        gpio_isr_handler_remove(ENCODER_CHANNEL_A_PIN);
        return ret;
    }
    
    return ESP_OK;
}

static esp_err_t encoder_setup_speed_timer(void) {
    esp_timer_create_args_t timer_args = {
        .callback = speed_calculation_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "encoder_speed_timer"
    };
    
    esp_err_t ret = esp_timer_create(&timer_args, &speed_calc_timer);
    if (ret != ESP_OK) {
        return ret;
    }
    
    return esp_timer_start_periodic(speed_calc_timer, ENCODER_SPEED_CALC_INTERVAL_US);
}

static void IRAM_ATTR encoder_channel_a_isr(void *arg) {
    uint8_t channel_a = gpio_get_level(ENCODER_CHANNEL_A_PIN);
    uint8_t channel_b = gpio_get_level(ENCODER_CHANNEL_B_PIN);
    encoder_update_position_and_direction(channel_a, channel_b);
}

static void IRAM_ATTR encoder_channel_b_isr(void *arg) {
    uint8_t channel_a = gpio_get_level(ENCODER_CHANNEL_A_PIN);
    uint8_t channel_b = gpio_get_level(ENCODER_CHANNEL_B_PIN);
    encoder_update_position_and_direction(channel_a, channel_b);
}

static void encoder_update_position_and_direction(uint8_t channel_a, uint8_t channel_b) {
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
    
    static uint32_t interrupt_count = 0;
    interrupt_count++;
    
    if (delta != 0) {
        encoder_position += delta;
        // ESP_LOGI(TAG, "Position: %d", encoder_position);
        last_pulse_time_us = esp_timer_get_time();
        
        if (delta > 0) {
            encoder_direction = ENCODER_DIRECTION_CLOCKWISE;
        } else {
            encoder_direction = ENCODER_DIRECTION_COUNTER_CLOCKWISE;
        }
    }
    
    last_channel_a = channel_a;
    last_channel_b = channel_b;
}

static void speed_calculation_timer_callback(void *arg) {
    static uint32_t last_calc_time_us = 0;
    static int64_t last_position = 0;
    
    uint32_t current_time_us = esp_timer_get_time();
    
    if (last_calc_time_us == 0) {
        last_calc_time_us = current_time_us;
        last_position = encoder_position;
        return;
    }
    
    uint32_t time_diff_us = current_time_us - last_calc_time_us;
    int64_t position_diff = encoder_position - last_position;
    
    if (time_diff_us > 0) {
        float revolutions = (float)position_diff / ENCODER_COUNTS_PER_REV;
        float time_diff_minutes = (float)time_diff_us / 60000000.0f;
        current_speed_rpm = revolutions / time_diff_minutes;
    }
    
    uint32_t time_since_last_pulse = current_time_us - last_pulse_time_us;
    if (time_since_last_pulse > 500000) {
        current_speed_rpm = 0.0f;
        encoder_direction = ENCODER_DIRECTION_UNKNOWN;
    }
    
    last_calc_time_us = current_time_us;
    last_position = encoder_position;
}

void encoder_reset_position(void) {
    if (xSemaphoreTake(encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        encoder_position = 0;
        current_speed_rpm = 0.0f;
        encoder_direction = ENCODER_DIRECTION_UNKNOWN;
        xSemaphoreGive(encoder_mutex);
        ESP_LOGI(TAG, "Encoder position reset to 0");
    }
}

int64_t encoder_get_position(void) {
    int64_t position = 0;
    if (xSemaphoreTake(encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        position = encoder_position;
        xSemaphoreGive(encoder_mutex);
    }
    return position;
}

float encoder_get_speed_rpm(void) {
    float speed = 0.0f;
    if (xSemaphoreTake(encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        speed = current_speed_rpm;
        xSemaphoreGive(encoder_mutex);
    }
    return speed;
}

encoder_direction_t encoder_get_direction(void) {
    encoder_direction_t direction = ENCODER_DIRECTION_UNKNOWN;
    if (xSemaphoreTake(encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        direction = encoder_direction;
        xSemaphoreGive(encoder_mutex);
    }
    return direction;
}

encoder_data_t encoder_get_data(void) {
    encoder_data_t data = {0};
    if (xSemaphoreTake(encoder_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        data.position = encoder_position;
        data.speed_rpm = current_speed_rpm;
        data.direction = encoder_direction;
        data.last_pulse_time_us = last_pulse_time_us;
        xSemaphoreGive(encoder_mutex);
    }
    return data;
}

float encoder_position_to_degrees(int64_t position) {
    return (float)position * 360.0f / ENCODER_COUNTS_PER_REV;
}

float encoder_position_to_radians(int64_t position) {
    return (float)position * 2.0f * M_PI / ENCODER_COUNTS_PER_REV;
}

int64_t encoder_degrees_to_position(float degrees) {
    return (int64_t)(degrees * ENCODER_COUNTS_PER_REV / 360.0f);
}

int64_t encoder_radians_to_position(float radians) {
    return (int64_t)(radians * ENCODER_COUNTS_PER_REV / (2.0f * M_PI));
}

void encoder_debug_pins(void) {
    uint8_t pin_a = gpio_get_level(ENCODER_CHANNEL_A_PIN);
    uint8_t pin_b = gpio_get_level(ENCODER_CHANNEL_B_PIN);
    ESP_LOGI(TAG, "Debug: Channel A=%d, Channel B=%d, Position=%lld", pin_a, pin_b, encoder_position);
}