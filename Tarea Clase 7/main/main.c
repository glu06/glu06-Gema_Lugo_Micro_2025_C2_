#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "nvs_flash.h"

// ---- ADC (oneshot + calibración) ----
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ---- GPTimer para muestreo ----
#include "driver/gptimer.h"

// ---- PWM (LEDC) para motor y buzzer ----
#include "driver/ledc.h"

// ================== CONFIG ==================
#define TAG                 "TAREA7"

#define ADC_UNIT_ID         ADC_UNIT_1
#define ADC_CHANNEL_ID      ADC_CHANNEL_6     // GPIO34
#define ADC_ATTEN           ADC_ATTEN_DB_11   // ~0-3.3V
#define ADC_BITW            ADC_BITWIDTH_DEFAULT

#define FS_HZ               2400              // Frecuencia de muestreo
#define RMS_WINDOW_S        1                 // Ventana de RMS en segundos

#define MOTOR_PWM_GPIO      25
#define BUZZER_PWM_GPIO     26
#define MOTOR_PWM_FREQ_HZ   20000             // 20 kHz (silencioso para motor DC)
#define BUZZER_PWM_FREQ_HZ  2000              // 2 kHz audible
#define PWM_RES             LEDC_TIMER_10_BIT // 1024 niveles

// ===========================================

static QueueHandle_t s_tick_queue;

// --------- ADC handles ---------
static adc_oneshot_unit_handle_t s_adc = NULL;
static adc_cali_handle_t s_adc_cali = NULL;
static bool s_do_cali = false;

// --------- GPTimer ISR ----------
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer,
                                        const gptimer_alarm_event_data_t *edata,
                                        void *user_ctx)
{
    BaseType_t hp_task_woken = pdFALSE;
    uint8_t tick = 1;
    xQueueSendFromISR(s_tick_queue, &tick, &hp_task_woken);
    return hp_task_woken == pdTRUE;
}

// --------- ADC init ----------
static void adc_init(void)
{
    // Unidad
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id  = ADC_UNIT_ID,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&unit_cfg, &s_adc));

    // Canal
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITW,
        .atten    = ADC_ATTEN
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc, ADC_CHANNEL_ID, &chan_cfg));

    // Calibración (si es posible)
    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_ID,
        .atten    = ADC_ATTEN,
        .bitwidth = ADC_BITW
    };
    if (adc_cali_create_scheme_line_fitting(&cali_cfg, &s_adc_cali) == ESP_OK) {
        s_do_cali = true;
        ESP_LOGI(TAG, "Calibración ADC activada (line fitting).");
    } else {
        s_do_cali = false;
        ESP_LOGW(TAG, "Calibración ADC NO disponible; se usará valor bruto.");
    }
}

// --------- GPTimer @ 2400 Hz ----------
static gptimer_handle_t timer_start_2400hz(void)
{
    gptimer_handle_t gpt;
    // Elegimos resolución 2.4 MHz => alarma cada 1000 cuentas = 2400 Hz exactos
    gptimer_config_t tcfg = {
        .clk_src       = GPTIMER_CLK_SRC_DEFAULT,
        .direction     = GPTIMER_COUNT_UP,
        .resolution_hz = 2400000
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&tcfg, &gpt));

    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gpt, &cbs, NULL));

    ESP_ERROR_CHECK(gptimer_enable(gpt));
    gptimer_alarm_config_t acfg = {
        .reload_count = 0,
        .alarm_count  = 1000,   // 2.4 MHz / 1000 = 2400 Hz
        .flags.auto_reload_on_alarm = true
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gpt, &acfg));
    ESP_ERROR_CHECK(gptimer_start(gpt));

    ESP_LOGI(TAG, "GPTimer iniciado a %d Hz", FS_HZ);
    return gpt;
}

// --------- PWM (LEDC) ----------
static void pwm_init(void)
{
    // MOTOR
    ledc_timer_config_t t_motor = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RES,
        .freq_hz          = MOTOR_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t_motor));

    ledc_channel_config_t ch_motor = {
        .gpio_num   = MOTOR_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_motor));

    // BUZZER
    ledc_timer_config_t t_buzzer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .timer_num        = LEDC_TIMER_1,
        .duty_resolution  = PWM_RES,
        .freq_hz          = BUZZER_PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t_buzzer));

    ledc_channel_config_t ch_buzzer = {
        .gpio_num   = BUZZER_PWM_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_1,
        .timer_sel  = LEDC_TIMER_1,
        .duty       = 0,
        .hpoint     = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_buzzer));
}

static void motor_pwm_set_percent(int percent)
{
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    uint32_t max_duty = (1U << PWM_RES) - 1U;
    uint32_t duty = (uint32_t)((percent / 100.0f) * max_duty);
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));
}

static void buzzer_set_50(bool on)
{
    uint32_t max_duty = (1U << PWM_RES) - 1U;
    uint32_t duty = on ? (max_duty / 2U) : 0U;   // 50% o 0%
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

// --------- Tarea de muestreo + RMS ----------
static void sampling_task(void *arg)
{
    const uint32_t samples_per_window = FS_HZ * RMS_WINDOW_S;

    uint32_t n = 0;
    double sumsq_raw = 0.0;
    double sumsq_mv  = 0.0;

    while (1) {
        uint8_t tick;
        if (xQueueReceive(s_tick_queue, &tick, portMAX_DELAY) == pdTRUE) {
            int raw = 0;
            ESP_ERROR_CHECK(adc_oneshot_read(s_adc, ADC_CHANNEL_ID, &raw));

            int mv = 0;
            if (s_do_cali) {
                ESP_ERROR_CHECK(adc_cali_raw_to_voltage(s_adc_cali, raw, &mv));
            } else {
                // Aproximación si no hay calibración (para 11dB y 12 bits)
                // ¡Mejor habilitar calibración para resultado real!
                mv = (int)((raw / 4095.0) * 3300.0);
            }

            sumsq_raw += (double)raw * (double)raw;
            sumsq_mv  += (double)mv  * (double)mv;
            n++;

            if (n >= samples_per_window) {
                double rms_raw = sqrt(sumsq_raw / (double)n);
                double rms_mv  = sqrt(sumsq_mv  / (double)n);
                ESP_LOGI(TAG, "RMS %lus: RAW=%.1f / %d bits,  V=%.3f V  (n=%" PRIu32 ")",
                         (unsigned long)RMS_WINDOW_S,
                         rms_raw, 12, rms_mv/1000.0, n);
                // reiniciar ventana
                n = 0;
                sumsq_raw = 0.0;
                sumsq_mv  = 0.0;
            }
        }
    }
}

// ---------------- app_main ----------------
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    // PWM motor/buzzer
    pwm_init();
    motor_pwm_set_percent(60);   // Motor al 60% para probar
    buzzer_set_50(true);         // Buzzer 50% duty a 2 kHz (encendido)
    vTaskDelay(pdMS_TO_TICKS(700)); // Suena un rato
    buzzer_set_50(false);        // Apaga

    // ADC + Timer + Tarea de muestreo
    adc_init();

    s_tick_queue = xQueueCreate(64, sizeof(uint8_t));
    (void)timer_start_2400hz();

    xTaskCreate(sampling_task, "sampling", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Sistema iniciado: ADC 2400 Hz, RMS 1 s, PWM motor y buzzer listo.");
}
