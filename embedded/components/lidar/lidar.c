#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "lidar_protocal.h"
#include "telemetry.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define UART_PORT 1
#define PIN_RX 16
#define PIN_TX 17
#define LIDAR_CTRL_MOTO 32

#define UART_BUFFER_SIZE 1024
#define UART_BAUD_RATE 115200

const int uart_buffer_size = 1024;

QueueHandle_t uart_event_queue;
QueueHandle_t g_lidar_data_queue;

uint8_t LidarData[1024];

static const char *TAG = "LIDAR";

void lidar_uart_init(void) {
    
    // Configure lidar motor control pin to high
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << LIDAR_CTRL_MOTO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&config);
    gpio_set_level(LIDAR_CTRL_MOTO, 1);


    /* 8-N-1 : 8 data bits, No parity bit, 1 stop bit*/
    uart_config_t uart_config = {
        .baud_rate  = UART_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE, 
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(UART_PORT, PIN_TX, PIN_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Delay to start lidar motor
    vTaskDelay(pdMS_TO_TICKS(500));

    // Send start message to lidar to begin scanning
    uint8_t start_scan_msg[2] = {0xA5, 0x20};
    uart_write_bytes(UART_PORT, start_scan_msg, sizeof(start_scan_msg));


    // Wait for 7 bytes response
    uint8_t scan_response[7];

    int bytes_read = uart_read_bytes(UART_PORT, scan_response, sizeof(scan_response), pdMS_TO_TICKS(1000));

    if (bytes_read == 7) {
        ESP_LOGI(TAG,
            "Descriptor: %02X %02X %02X %02X %02X %02X %02X",
            scan_response[0],
            scan_response[1],
            scan_response[2],
            scan_response[3],
            scan_response[4],
            scan_response[5],
            scan_response[6]
        );
    } else {
        ESP_LOGE(TAG, "Expected 7 bytes, recieved %d", bytes_read);
    }

}

void rx_lidar_task(void *arg) {

    telemetry_lidar_msg_t lidar_msg = {0};
    uint16_t batch_count = 0;

    uint32_t total_samples = 0;
    uint32_t completed_scans = 0;

    for(;;) {

        uart_event_t event;

        // Sleep until a uart event is received
        if (xQueueReceive(uart_event_queue, &event, portMAX_DELAY)) {

            switch (event.type)
            {
            case UART_DATA:
                
                size_t len = 0;
                ESP_ERROR_CHECK(uart_get_buffered_data_len(UART_PORT, &len));

                if (len > sizeof(LidarData)) {
                    len = sizeof(LidarData);
                }
                
                // If the uart buffer has data transfer that data into the Lidar Data buffer
                if (len > 0) {
                    int bytes_read = uart_read_bytes(UART_PORT, &LidarData, len, 0);
                    telemetry_lidar_t sample;

                    for (int i = 0; i < bytes_read; i++) {
                        
                        if (parse_byte(LidarData[i], &sample)) {

                            total_samples++;
                            

                            if (sample.start_flag) {

                                completed_scans++;

                                ESP_LOGI(TAG, "New scan #%lu | total samples=%lu", (unsigned long)completed_scans, (unsigned long)total_samples);
                            }
                            
                            /* If it is the first sample in the batch get the time*/
                            if (batch_count == 0) {
                                lidar_msg.header.timestamp_us = esp_timer_get_time();
                            }

                            lidar_msg.lidar_batch[batch_count] = sample;

                            batch_count++;
                            
                            /* If the batch count is at the max, enqueue the batch and rest the count */
                            if (batch_count == LIDAR_BATCH_SIZE) {

                                lidar_msg.batch_count = batch_count;
                                
                                telemetry_enqueue_data(&lidar_msg, TELEMETRY_LIDAR);

                                batch_count = 0;

                            }
                        }
                }
                
                }
                break;
            
            default:
                break;
            }
        }
    }
}
