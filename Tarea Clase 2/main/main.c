
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <inttypes.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "mqtt_client.h"

// =================== CONFIG ===================
// WiFi/MQTT
#define WIFI_SSID       "TU_SSID"
#define WIFI_PASS       "TU_PASSWORD"
#define MQTT_URI        "mqtt://broker.hivemq.com:1883"
#define TOPIC_BASE      "esp32/pp"
#define TOPIC_STATE     TOPIC_BASE "/state"
#define TOPIC_CMD       TOPIC_BASE "/cmd"

// Pines (ajústalos a tu hardware)
#define BTN_PP_GPIO         GPIO_NUM_4      // Botón con PULLUP, activo en 0
#define LAMP_GPIO           GPIO_NUM_2      // LED/Lámpara
#define BUZZER_GPIO         GPIO_NUM_26     // Buzzer con PWM LEDC

#define LIM_OPEN_GPIO       GPIO_NUM_33     // Fin de carrera "abierto"
#define LIM_CLOSE_GPIO      GPIO_NUM_32     // Fin de carrera "cerrado"
#define SENSOR_ACTIVE_LEVEL 1               // 1 si el sensor entrega HIGH al activar

// Motor (dos señales: abrir/cerrar, tipo relé o driver)
#define MOTOR_OPEN_GPIO     GPIO_NUM_27
#define MOTOR_CLOSE_GPIO    GPIO_NUM_19

// Tiempos
#define TICK_MS             50              // periodo de sistema
#define OPEN_TIMEOUT_MS     12000           // 12 s para abrir/cerrar
#define BLINK_OPEN_MS       500             // parpadeo en apertura
#define BLINK_CLOSE_MS      250             // parpadeo en cierre

// PWM buzzer
#define BUZZER_FREQ_HZ      2000
#define PWM_RES             LEDC_TIMER_10_BIT

// ==============================================
static const char *TAG = "TAREA2";

// ---- Helpers tiempo->ticks ----
#define MS_TO_TICKS(ms)     ((ms) / TICK_MS)

// ---- Timer 50ms ----
static QueueHandle_t s_tick_queue;
static void periodic_cb(void *arg) {
    uint8_t one = 1;
    if (s_tick_queue) xQueueSend(s_tick_queue, &one, 0);
}

// ---- MQTT ----
static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;

// ---- Sensores ----
static inline bool lim_open_active(void)  { return gpio_get_level(LIM_OPEN_GPIO)  == SENSOR_ACTIVE_LEVEL; }
static inline bool lim_close_active(void) { return gpio_get_level(LIM_CLOSE_GPIO) == SENSOR_ACTIVE_LEVEL; }

// ---- Actuadores ----
static inline void motor_stop(void)        { gpio_set_level(MOTOR_OPEN_GPIO, 0); gpio_set_level(MOTOR_CLOSE_GPIO, 0); }
static inline void motor_open_dir(void)    { gpio_set_level(MOTOR_CLOSE_GPIO, 0); gpio_set_level(MOTOR_OPEN_GPIO, 1); }
static inline void motor_close_dir(void)   { gpio_set_level(MOTOR_OPEN_GPIO, 0);  gpio_set_level(MOTOR_CLOSE_GPIO, 1); }
static inline void lamp_on(bool on)        { gpio_set_level(LAMP_GPIO, on ? 1 : 0); }

// Buzzer con LEDC 50% duty
static inline void buzzer_on(void)  {
    uint32_t max_duty = (1U<<PWM_RES) - 1U;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, max_duty/2);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}
static inline void buzzer_off(void) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

// ---- Estados ----
typedef enum {
    ST_UNKNOWN = 0,
    ST_CLOSED,
    ST_OPENING,
    ST_OPEN,
    ST_CLOSING,
    ST_ERROR
} pp_state_t;

typedef enum {
    ERR_NONE = 0,
    ERR_LIMIT_CONFLICT = 1,   // ambos finales activos
    ERR_TIMEOUT_OPEN   = 2,
    ERR_TIMEOUT_CLOSE  = 3
} pp_error_t;

static volatile pp_state_t s_state = ST_UNKNOWN;
static volatile pp_error_t s_error = ERR_NONE;

// ---- Botón con anti-rebote ----
static bool btn_pressed_edge(void) {
    static uint8_t stable_cnt = 0;
    static int last_raw = 1;         // pull-up => reposo HIGH
    static int debounced = 1;
    static int last_debounced = 1;

    int raw = gpio_get_level(BTN_PP_GPIO);
    if (raw == last_raw) stable_cnt++; else { stable_cnt = 0; last_raw = raw; }

    if (stable_cnt >= 2) {           // ~100 ms de estabilidad
        debounced = raw;
        stable_cnt = 0;
    }

    bool edge = (last_debounced == 1 && debounced == 0); // flanco de bajada => pulsado
    last_debounced = debounced;
    return edge;
}

// ---- Publicación estado ----
static void publish_state(const char *reason) {
    if (!s_mqtt_connected) return;
    char json[160];
    snprintf(json, sizeof(json),
        "{\"state\":\"%s\",\"error\":%d,\"reason\":\"%s\"}",
        (s_state==ST_UNKNOWN)?"UNKNOWN":
        (s_state==ST_CLOSED) ?"CLOSED":
        (s_state==ST_OPENING)?"OPENING":
        (s_state==ST_OPEN)   ?"OPEN":
        (s_state==ST_CLOSING)?"CLOSING":"ERROR",
        s_error, reason?reason:"");
    esp_mqtt_client_publish(s_mqtt, TOPIC_STATE, json, 0, 1, 0);
}

// ---- MQTT handler ----
static void mqtt_event(void *handler_args, esp_event_base_t base, int32_t id, void *data) {
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)data;
    switch ((esp_mqtt_event_id_t)id) {
        case MQTT_EVENT_CONNECTED:
            s_mqtt_connected = true;
            esp_mqtt_client_subscribe(s_mqtt, TOPIC_CMD, 1);
            esp_mqtt_client_publish(s_mqtt, TOPIC_STATE, "{\"boot\":\"online\"}", 0, 1, 0);
            break;
        case MQTT_EVENT_DISCONNECTED:
            s_mqtt_connected = false;
            break;
        case MQTT_EVENT_DATA: {
            char topic[64]={0}, payload[64]={0};
            int tlen = e->topic_len < 63 ? e->topic_len : 63;
            int dlen = e->data_len  < 63 ? e->data_len  : 63;
            memcpy(topic, e->topic, tlen); topic[tlen]=0;
            memcpy(payload, e->data, dlen); payload[dlen]=0;

            if (strcmp(topic, TOPIC_CMD)==0) {
                // comandos: open | close | stop | lamp:on | lamp:off | reset
                if (strcasecmp(payload,"open")==0) {
                    if (s_state==ST_CLOSED || s_state==ST_UNKNOWN) { s_state = ST_OPENING; publish_state("cmd:open"); }
                } else if (strcasecmp(payload,"close")==0) {
                    if (s_state==ST_OPEN || s_state==ST_UNKNOWN) { s_state = ST_CLOSING; publish_state("cmd:close"); }
                } else if (strcasecmp(payload,"stop")==0) {
                    motor_stop(); s_state = ST_UNKNOWN; publish_state("cmd:stop");
                } else if (strcasecmp(payload,"lamp:on")==0) {
                    lamp_on(true);
                } else if (strcasecmp(payload,"lamp:off")==0) {
                    lamp_on(false);
                } else if (strcasecmp(payload,"reset")==0) {
                    s_error = ERR_NONE; s_state = ST_UNKNOWN; publish_state("cmd:reset");
                }
            }
            break;
        }
        default: break;
    }
}

static void wifi_start(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wcfg = {0};
    strlcpy((char*)wcfg.sta.ssid, WIFI_SSID, sizeof(wcfg.sta.ssid));
    strlcpy((char*)wcfg.sta.password, WIFI_PASS, sizeof(wcfg.sta.password));
    wcfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wcfg));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "Conectando WiFi...");
}

static void mqtt_start(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.client_id = "esp32-pp",
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
        .session.protocol_ver = MQTT_PROTOCOL_V_3_1_1,
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event, NULL);
    esp_mqtt_client_start(s_mqtt);
}

// ---- FSM tarea principal (tick 50ms) ----
static void fsm_task(void *arg) {
    uint32_t t_blink = 0;
    uint32_t t_timeout = 0;
    bool lamp_level = false;
    bool buzzer_level = false;
    uint32_t buz_cnt = 0;

    // Estado inicial por sensores
    if (lim_open_active() && lim_close_active()) { s_state = ST_ERROR; s_error = ERR_LIMIT_CONFLICT; }
    else if (lim_open_active())                   { s_state = ST_OPEN;  }
    else if (lim_close_active())                  { s_state = ST_CLOSED;}
    else                                          { s_state = ST_UNKNOWN;}
    publish_state("boot");

    while (1) {
        uint8_t one;
        if (xQueueReceive(s_tick_queue, &one, portMAX_DELAY) != pdTRUE) continue;

        // Botón: lógica solicitada
        if (btn_pressed_edge()) {
            if (s_state == ST_CLOSED)      { s_state = ST_OPENING;  publish_state("btn->open"); }
            else if (s_state == ST_OPEN)   { s_state = ST_CLOSING;  publish_state("btn->close"); }
            else if (s_state == ST_UNKNOWN){ s_state = ST_CLOSING;  publish_state("btn->unknown->close"); }
        }

        // Buzzer en error (beep 5 Hz)
        if (s_state == ST_ERROR) {
            buz_cnt++;
            if (buzzer_level) buzzer_on(); else buzzer_off();
            if (buz_cnt >= MS_TO_TICKS(100)) { buz_cnt = 0; buzzer_level = !buzzer_level; } // 100ms ON/OFF
        } else {
            buzzer_off(); buzzer_level=false; buz_cnt=0;
        }

        // Máquina de estados
        switch (s_state) {
            case ST_OPENING:
                // seguridad
                if (lim_open_active() && lim_close_active()) { s_state = ST_ERROR; s_error = ERR_LIMIT_CONFLICT; motor_stop(); publish_state("err:limits"); break; }
                // acción
                motor_open_dir();
                t_timeout++;
                // lámpara 0.5s
                if (++t_blink >= MS_TO_TICKS(BLINK_OPEN_MS)) { t_blink = 0; lamp_level = !lamp_level; lamp_on(lamp_level); }
                // fin de carrera abierto
                if (lim_open_active()) { motor_stop(); s_state = ST_OPEN; t_timeout = 0; lamp_on(true); publish_state("opened"); }
                // timeout
                else if (t_timeout >= MS_TO_TICKS(OPEN_TIMEOUT_MS)) { motor_stop(); s_state = ST_ERROR; s_error = ERR_TIMEOUT_OPEN; publish_state("err:timeout_open"); }
                break;

            case ST_CLOSING:
                if (lim_open_active() && lim_close_active()) { s_state = ST_ERROR; s_error = ERR_LIMIT_CONFLICT; motor_stop(); publish_state("err:limits"); break; }
                motor_close_dir();
                t_timeout++;
                // lámpara 0.25s
                if (++t_blink >= MS_TO_TICKS(BLINK_CLOSE_MS)) { t_blink = 0; lamp_level = !lamp_level; lamp_on(lamp_level); }
                if (lim_close_active()) { motor_stop(); s_state = ST_CLOSED; t_timeout = 0; lamp_on(false); publish_state("closed"); }
                else if (t_timeout >= MS_TO_TICKS(OPEN_TIMEOUT_MS)) { motor_stop(); s_state = ST_ERROR; s_error = ERR_TIMEOUT_CLOSE; publish_state("err:timeout_close"); }
                break;

            case ST_OPEN:
                lamp_on(true);
                motor_stop();
                t_blink = 0; t_timeout = 0;
                break;

            case ST_CLOSED:
                lamp_on(false);
                motor_stop();
                t_blink = 0; t_timeout = 0;
                break;

            case ST_UNKNOWN:
                // queda esperando orden; lámpara apagada
                lamp_on(false);
                motor_stop();
                t_blink = 0; t_timeout = 0;
                break;

            case ST_ERROR:
                motor_stop();
                // lámpara intermitente lenta para llamar la atención
                if (++t_blink >= MS_TO_TICKS(500)) { t_blink = 0; lamp_level = !lamp_level; lamp_on(lamp_level); }
                break;
        }
    }
}

void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // GPIO
    gpio_config_t out = {
        .pin_bit_mask = (1ULL<<LAMP_GPIO) | (1ULL<<MOTOR_OPEN_GPIO) | (1ULL<<MOTOR_CLOSE_GPIO),
        .mode = GPIO_MODE_OUTPUT, .pull_up_en = 0, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out);
    lamp_on(false); motor_stop();

    gpio_config_t in_btn = {
        .pin_bit_mask = 1ULL<<BTN_PP_GPIO, .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_btn);

    gpio_config_t in_limits = {
        .pin_bit_mask = (1ULL<<LIM_OPEN_GPIO) | (1ULL<<LIM_CLOSE_GPIO),
        .mode = GPIO_MODE_INPUT, .pull_up_en = 1, .pull_down_en = 0, .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&in_limits);

    // PWM buzzer
    ledc_timer_config_t t = { .speed_mode=LEDC_LOW_SPEED_MODE, .timer_num=LEDC_TIMER_0,
                              .duty_resolution=PWM_RES, .freq_hz=BUZZER_FREQ_HZ, .clk_cfg=LEDC_AUTO_CLK };
    ledc_timer_config(&t);
    ledc_channel_config_t ch = { .gpio_num=BUZZER_GPIO, .speed_mode=LEDC_LOW_SPEED_MODE,
                                 .channel=LEDC_CHANNEL_0, .timer_sel=LEDC_TIMER_0, .duty=0, .hpoint=0 };
    ledc_channel_config(&ch);

    // WiFi/MQTT
    wifi_start();
    mqtt_start();

    // Timer 50 ms
    s_tick_queue = xQueueCreate(32, sizeof(uint8_t));
    esp_timer_create_args_t args = { .callback = periodic_cb, .name = "tick50ms" };
    esp_timer_handle_t timer; esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, TICK_MS * 1000); // us

    // FSM
    xTaskCreate(fsm_task, "pp_fsm", 4096, NULL, 6, NULL);

    ESP_LOGI(TAG, "Sistema listo (Tarea Clase 2). Topics: %s (estado), %s (comandos).",
             TOPIC_STATE, TOPIC_CMD);
}
