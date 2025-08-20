#include <stdio.h>

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "esp_timer.h"

// Pin Definitions
#define LED_STATUS_PIN      2
#define LED_RED_PIN         33
#define LED_GREEN_PIN       25
#define LED_BLUE_PIN        26
#define ADC_CHANNEL         ADC1_CHANNEL_4

// Configuration Constants
#define SAMPLE_PERIOD_US    416
#define ADC_MAX_VAL         4095

// Static variables
static const char *TAG = "REF_CODE";
static uint8_t status_led_state = 0;

// Function Prototypes
static void configure_gpio(void);
static void configure_adc(void);
static void configure_high_res_timer(void);
static void timer_callback_handler(void *arg);
static void update_led_status(void);
static void control_rgb_leds(int adc_raw_value);

// Main application entry point
void app_main(void)
{
    // Initialize components
    configure_gpio();
    configure_adc();
    configure_high_res_timer();

    // Set log level and display start message
    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "Application started and components configured");
}

// GPIO Initialization
static void configure_gpio(void)
{
    gpio_set_direction(LED_STATUS_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_RED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_GREEN_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_BLUE_PIN, GPIO_MODE_OUTPUT);
    ESP_LOGI(TAG, "GPIO pins for LEDs configured successfully");
}

// ADC Configuration
static void configure_adc(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
    ESP_LOGI(TAG, "ADC configured on channel %d", ADC_CHANNEL);
}

// High-resolution timer setup
static void configure_high_res_timer(void)
{
    esp_timer_handle_t adc_timer;
    const esp_timer_create_args_t timer_args = {
        .callback = &timer_callback_handler,
        .name = "adc_sampler"
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &adc_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(adc_timer, SAMPLE_PERIOD_US));
    ESP_LOGI(TAG, "High-res timer started with a %d us period", SAMPLE_PERIOD_US);
}

// Timer interrupt callback
static void timer_callback_handler(void *arg)
{
    // Toggle the status LED
    update_led_status();

    // Read and log the ADC value
    int adc_reading = adc1_get_raw(ADC_CHANNEL);
    ESP_LOGI(TAG, "ADC Reading: %i", adc_reading);

    // Control RGB LEDs based on the ADC reading
    control_rgb_leds(adc_reading);
}

// Helper function to toggle the status LED
static void update_led_status(void)
{
    status_led_state = !status_led_state;
    gpio_set_level(LED_STATUS_PIN, status_led_state);
}

// Helper function to manage the RGB LED states
static void control_rgb_leds(int adc_raw_value)
{
    int threshold = adc_raw_value / 1000;

    // Reset all LEDs to OFF by default
    gpio_set_level(LED_RED_PIN, 0);
    gpio_set_level(LED_GREEN_PIN, 0);
    gpio_set_level(LED_BLUE_PIN, 0);

    // Turn on specific LEDs based on the threshold
    switch (threshold) {
        case 1:
            gpio_set_level(LED_RED_PIN, 1);
            break;
        case 2:
            gpio_set_level(LED_GREEN_PIN, 1);
            break;
        case 3:
        case 4:
            gpio_set_level(LED_RED_PIN, 1);
            gpio_set_level(LED_GREEN_PIN, 1);
            gpio_set_level(LED_BLUE_PIN, 1);
            break;
        default:
            // All LEDs remain OFF (handled by the reset)
            break;
    }
}