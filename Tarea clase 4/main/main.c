
#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#define LED_GPIO  GPIO_NUM_2
static const char *TAG = "TAREA4";

static TimerHandle_t t_blink;   
static TimerHandle_t t_once;   

static void cb_blink(TimerHandle_t xTimer)
{
    static bool level = false;
    level = !level;
    gpio_set_level(LED_GPIO, level);
    static uint32_t cnt = 0;
    ESP_LOGI(TAG, "Blink! count=%" PRIu32, ++cnt);
}

static void cb_once(TimerHandle_t xTimer)
{
    ESP_LOGI(TAG, "One-shot disparado a los 3 segundos.");
}

void app_main(void)
{
    // NVS 
    ESP_ERROR_CHECK(nvs_flash_init());

    
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    gpio_set_level(LED_GPIO, 0);

    
    t_blink = xTimerCreate("blink", pdMS_TO_TICKS(500), pdTRUE,  NULL, cb_blink); 
    t_once  = xTimerCreate("once",  pdMS_TO_TICKS(3000), pdFALSE, NULL, cb_once);

    // Iniciar
    xTimerStart(t_blink, 0);
    xTimerStart(t_once,  0);

    ESP_LOGI(TAG, "Timers iniciados: blink 500 ms (periodico), one-shot 3 s.");
}
