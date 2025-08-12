#include "driver/ledc.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

const int A1_PIN = GPIO_NUM_12;
const int B1_PIN = GPIO_NUM_13;
const int A2_PIN = GPIO_NUM_14;
const int B2_PIN = GPIO_NUM_15;

const int PWM_PIN_1 = GPIO_NUM_26;
const int PWM_PIN_2 = GPIO_NUM_27;

const long int bit_mask = (1ULL << A1_PIN) | (1ULL << B1_PIN) |
                          (1ULL << A2_PIN) | (1ULL << B2_PIN);

const char *MAIN = "MAIN";

#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_CHANNEL_1 LEDC_CHANNEL_0
#define LEDC_CHANNEL_2 LEDC_CHANNEL_1
#define LEDC_DUTY_RES LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY 60

void ledc_setup();
void clock_wise_spin();

// defines 
const int LOW = 0;
const int HIGH = 1;

void app_main(void)
{
    gpio_config_t gpio_conf = {
        .pin_bit_mask = bit_mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};

    esp_err_t ret = gpio_config(&gpio_conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE(MAIN, "GPIO not initialize correctly");
    };

    ledc_setup();

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_1, 128); 
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_1);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_2, 128); 
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_2);

    while (1)
    {
        clock_wise_spin();
    }
}

void ledc_setup()
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel_1 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_PIN_1,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&ledc_channel_1);

    ledc_channel_config_t ledc_channel_2 = {
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = PWM_PIN_2,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&ledc_channel_2);
}

void clock_wise_spin()
{
    gpio_set_level(A1_PIN, HIGH);
    gpio_set_level(B1_PIN, LOW);
    gpio_set_level(A2_PIN, HIGH);
    gpio_set_level(B2_PIN, LOW);
    vTaskDelay(pdMS_TO_TICKS(17)); // 60 Hz -> ~16,67 ms

    // Passo 2
    gpio_set_level(A1_PIN, LOW);
    gpio_set_level(B1_PIN, HIGH);
    gpio_set_level(A2_PIN, LOW);
    gpio_set_level(B2_PIN, HIGH);
    vTaskDelay(pdMS_TO_TICKS(17));
}
