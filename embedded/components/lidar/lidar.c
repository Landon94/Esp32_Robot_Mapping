#include "driver/uart.h"

#define UART_PIN 1 //TOD fix
#define UART_BUFFER_SIZE 1024

const int uart_buffer_size = (1024 * 2);
QueueHandle_t uart_queue;


// void uart_init(void) {

//     ESP_ERROR_CHECK(uart_driver_install(UART_PIN, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 10, &uart_queue, 0));


//     uart_config_t uart_config = {
//         .baud_rate =,
//         .data_bits =,
//         .parity,
//         .stop_bits,
//         .flow_ctrl,
//         .rx_flow_ctrl_thresh
//     };

//     ESP_ERROR_CHECK(uart_param_config(UART_PIN, &uart_config));
// }


