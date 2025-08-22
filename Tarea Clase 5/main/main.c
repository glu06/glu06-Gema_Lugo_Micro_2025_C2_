#include <stdio.h>
#include <string.h>
#include <strings.h>  // strcasecmp
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"

#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "mqtt_client.h"

// =================== CONFIGURACIÓN WIFI ===================
#define WIFI_SSID       "TU_SSID"
#define WIFI_PASS       "TU_PASSWORD"

#define MQTT_URI        "mqtt://broker.hivemq.com:1883"

#define TOPIC_BASE      "esp32/curso/idf"
#define TOPIC_PUB       TOPIC_BASE "/status"
#define TOPIC_SUB       TOPIC_BASE "/led"

#define LED_GPIO        GPIO_NUM_2
// =====================================================

static const char *TAG = "APP";

static EventGroupHandle_t s_wifi_event_group;
static const int WIFI_CONNECTED_BIT = BIT0;

static esp_mqtt_client_handle_t s_mqtt = NULL;
static volatile bool s_mqtt_connected = false;  
static QueueHandle_t s_tick_queue;

typedef enum {
    ST_INIT = 0,
    ST_WIFI_CONNECTING,
    ST_WIFI_CONNECTED,
    ST_MQTT_CONNECTING,
    ST_MQTT_CONNECTED,
    ST_RUN
} app_state_t;

static volatile app_state_t s_state = ST_INIT;
static volatile bool s_led_on = false;

static void led_apply(bool on) {
    gpio_set_level(LED_GPIO, on ? 1 : 0);
}

// ---------------- Timer 50 ms ----------------
static void periodic_timer_cb(void *arg) {
    uint8_t tick = 1;
    if (s_tick_queue) xQueueSend(s_tick_queue, &tick, 0);
}

// ---------------- WiFi ----------------
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "WiFi: intentando conectar...");
        s_state = ST_WIFI_CONNECTING;
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi: desconectado, reintentando...");
        s_mqtt_connected = false;   // por si se cayó el broker
        xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
        s_state = ST_WIFI_CONNECTING;
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "WiFi: IP %d.%d.%d.%d",
                 IP2STR(&e->ip_info.ip));
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        s_state = ST_WIFI_CONNECTED;
    }
}

static void wifi_init_sta(void) {
    s_wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                    IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    strlcpy((char*)wifi_config.sta.ssid, WIFI_SSID, sizeof(wifi_config.sta.ssid));
    strlcpy((char*)wifi_config.sta.password, WIFI_PASS, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}

// ---------------- MQTT ----------------
static void mqtt_event_handler(void *handler_args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t) event_data;

    switch ((esp_mqtt_event_id_t)event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT: conectado");
            s_mqtt_connected = true;
            s_state = ST_MQTT_CONNECTED;

            esp_mqtt_client_subscribe(s_mqtt, TOPIC_SUB, 1);

            // Publicación inicial "online"
#if CONFIG_MQTT_PROTOCOL_5
            // Si activas MQTT 5.0 en menuconfig, se compila este bloque
            esp_mqtt5_property_list_t *props = esp_mqtt5_property_list_init();
            if (props) {
                esp_mqtt5_user_property_item_t up = { .key = "origin", .value = "esp32-devkit" };
                esp_mqtt5_property_list_add_user_property(props, &up);
                esp_mqtt_client_publish_with_properties(
                    s_mqtt, TOPIC_PUB, "online", 0, 1, 0, props);
                esp_mqtt5_property_list_destroy(props);
            } else {
                esp_mmqtt_client_publish(s_mqtt, TOPIC_PUB, "online", 0, 1, 0);
            }
#else
            esp_mqtt_client_publish(s_mqtt, TOPIC_PUB, "online", 0, 1, 0);
#endif
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT: desconectado");
            s_mqtt_connected = false;
            s_state = ST_WIFI_CONNECTED;
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT: suscrito a %s", TOPIC_SUB);
            break;

        case MQTT_EVENT_DATA: {
            char topic[128] = {0};
            char data[128] = {0};
            int tlen = event->topic_len < (int)sizeof(topic)-1 ? event->topic_len : (int)sizeof(topic)-1;
            int dlen = event->data_len  < (int)sizeof(data)-1  ? event->data_len  : (int)sizeof(data)-1;
            memcpy(topic, event->topic, tlen);
            memcpy(data,  event->data,  dlen);
            topic[tlen] = 0; data[dlen] = 0;

            ESP_LOGI(TAG, "MQTT RX: %s => %s", topic, data);
            if (strcmp(topic, TOPIC_SUB) == 0) {
                if (strcasecmp(data, "on") == 0)  s_led_on = true;
                if (strcasecmp(data, "off") == 0) s_led_on = false;
                led_apply(s_led_on);
            }
            break;
        }

        default:
            break;
    }
}

static void mqtt_start(void) {
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = MQTT_URI,
        .credentials.client_id = "esp32-devkit-fsm",
        .network.disable_auto_reconnect = false,
        .network.reconnect_timeout_ms = 5000,
        .session.protocol_ver = MQTT_PROTOCOL_V_5, // si MQTT5 no está habilitado, el driver usa 3.1.1
    };
    s_mqtt = esp_mqtt_client_init(&cfg);
    ESP_ERROR_CHECK(esp_mqtt_client_register_event(s_mqtt, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_mqtt_client_start(s_mqtt));
    s_state = ST_MQTT_CONNECTING;
}

// ------------- Máquina de estados -------------
static void state_machine_task(void *arg) {
    uint32_t tick_count = 0;     // ticks de 50 ms
    const uint32_t pub_1s_ticks = 20;  // 1000/50 = 20

    while (1) {
        uint8_t t;
        if (xQueueReceive(s_tick_queue, &t, portMAX_DELAY) == pdTRUE) {
            tick_count++;

            // Parpadeo en RUN cada 500 ms
            if (s_state == ST_RUN && (tick_count % 10) == 0) {
                s_led_on = !s_led_on;
                led_apply(s_led_on);
            }

            switch (s_state) {
                case ST_INIT:
                    s_state = ST_WIFI_CONNECTING;
                    break;

                case ST_WIFI_CONNECTED:
                    if (s_mqtt == NULL) {
                        mqtt_start();
                    } else if (s_mqtt_connected) {
                        s_state = ST_MQTT_CONNECTED;
                    }
                    break;

                case ST_MQTT_CONNECTED:
                    s_state = ST_RUN;
                    break;

                case ST_RUN:
                    if ((tick_count % pub_1s_ticks) == 0 && s_mqtt && s_mqtt_connected) {
                        char msg[64];
                        snprintf(msg, sizeof(msg), "{\"uptime_ms\":%" PRIu32 "}",
                                 (uint32_t)(esp_timer_get_time()/1000));
                        esp_mqtt_client_publish(s_mqtt, TOPIC_PUB, msg, 0, 1, 0);
                    }
                    break;

                default:
                    break;
            }
        }
    }
}

// ---------------- app_main ----------------
void app_main(void) {
    ESP_ERROR_CHECK(nvs_flash_init());

    // LED
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << LED_GPIO,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);
    led_apply(false);

    // WiFi
    wifi_init_sta();

    // Cola y timer 50 ms
    s_tick_queue = xQueueCreate(16, sizeof(uint8_t));
    esp_timer_create_args_t tcfg = {
        .callback = periodic_timer_cb,
        .arg = NULL,
        .name = "tick50ms"
    };
    esp_timer_handle_t timer;
    ESP_ERROR_CHECK(esp_timer_create(&tcfg, &timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(timer, 50 * 1000)); // 50 ms

    // FSM
    xTaskCreate(state_machine_task, "fsm", 4096, NULL, 5, NULL);

    ESP_LOGI(TAG, "Sistema iniciado: WiFi+MQTT, timer 50ms y FSM corriendo.");
}
