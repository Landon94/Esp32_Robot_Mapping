#include "esp_err.h"
#include "nvs_flash.h"
#include "wifi_control.h"
#include "web_control.h"
#include "motor.h"
#include "encoder.h"


void app_main(void) {
    
    nvs_flash_init();
    wifi_init_sta();
    start_web_server();
    motor_init();
    rpm_init();


    xTaskCreate(rpm_task, "rpm_task", 2048, NULL, 1, NULL);
    xTaskCreate(motor_control_task, "motor_control_task", 2048, NULL, 5, NULL);
}