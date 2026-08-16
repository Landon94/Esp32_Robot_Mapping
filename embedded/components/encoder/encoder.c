#include "driver/pulse_cnt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"


#define PHOTO_SENSOR_PIN_1 16
#define PHOTO_SENSOR_PIN_2 14

// Defined by number of slots on wheel encoder
#define PULSE_PER_REV 20 

static const char *TAG = "ENCODER";

static pcnt_unit_handle_t pcnt_unit_1 = NULL;
static pcnt_unit_handle_t pcnt_unit_2 = NULL;
static int64_t time_us = 0;

float g_rpm_1 = 0;
float g_rpm_2 = 0;

void rpm_init(void) {

    /* Specify range of internal hardware timer*/
    pcnt_unit_config_t unit_config = {
        .high_limit = 10000,
        .low_limit = -10000
    };

    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_1));
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit_2));

    /* Set pin number and edge level*/
    pcnt_chan_config_t pcnt_config_1 = {
        .edge_gpio_num = PHOTO_SENSOR_PIN_1,
        .level_gpio_num = -1                    // -1 means not used
    };

    pcnt_chan_config_t pcnt_config_2 = {
        .edge_gpio_num = PHOTO_SENSOR_PIN_2,
        .level_gpio_num = -1
    };


    pcnt_channel_handle_t pcnt_chan_1 = NULL;
    pcnt_channel_handle_t pcnt_chan_2 = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_1, &pcnt_config_1, &pcnt_chan_1));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit_2, &pcnt_config_2, &pcnt_chan_2));

    /* Set action for rising and falling edges*/
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_1, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_2, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_HOLD));


    /* Enable pins and start*/
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_1));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit_2));

    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_1));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_2));

    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_1));
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit_2));

    time_us = esp_timer_get_time();

}


void rpm_task(void *arg) {

    for (;;) {

        int count_1 = 0;
        int count_2 = 0;

        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_1, &count_1));
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit_2, &count_2));

        uint64_t time_ellapsed = esp_timer_get_time() - time_us;

        g_rpm_1 = ((float)count_1 * 60000.0f) / (PULSE_PER_REV * time_ellapsed);
        g_rpm_2 = ((float)count_2 * 60000.0f) / (PULSE_PER_REV * time_ellapsed);

        #if 1
            ESP_LOGI(TAG, "RPM 1: %.2f, RPM 2: %.2f", g_rpm_1, g_rpm_2);
        #endif

        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_1));
        ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit_2));

        time_us = esp_timer_get_time();

        /* Converts MS by multipling tick rate in HZ with specified ms time divided by 1000*/
        vTaskDelay(pdMS_TO_TICKS(250));
    }

}


/*(96237) ENCODER: RPM 1: 0.25, RPM 2: 0.18
I (96487) ENCODER: RPM 1: 0.24, RPM 2: 0.18
I (96737) ENCODER: RPM 1: 0.24, RPM 2: 0.17
I (96987) ENCODER: RPM 1: 0.24, RPM 2: 0.18
I (97237) ENCODER: RPM 1: 0.25, RPM 2: 0.18
I (97487) ENCODER: RPM 1: 0.24, RPM 2: 0.17
I (97737) ENCODER: RPM 1: 0.24, RPM 2: 0.18*/