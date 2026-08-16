#include "esp_wifi.h"
#include "esp_err.h"
#include "wifi_control.h"
#include "esp_log.h"
#include "sdkconfig.h"

// #define WIFI_SSID  CONFIG_ESP_WIFI_SSID
// #define WIFI_PASS  CONFIG_ESP_WIFI_PASSWORD

#define WIFI_SSID ""
#define WIFI_PASS ""
#define ESP_MAXIMUM_RETRY 3

static uint8_t retry_count = 0;
static const char *TAG = "web_server";

void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        ESP_LOGI(TAG, "Wifi successfully connected");
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (retry_count < ESP_MAXIMUM_RETRY){
            retry_count++;
            esp_wifi_connect();
        } else {
            //TODO
            ;
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP address: " IPSTR, IP2STR(&event->ip_info.ip));
        ESP_LOGI(TAG, "Access at http://" IPSTR "/", IP2STR(&event->ip_info.ip));
    }
}

void wifi_init_sta(void) {
    
    esp_netif_init();
    esp_event_loop_create_default();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS
        }
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
}