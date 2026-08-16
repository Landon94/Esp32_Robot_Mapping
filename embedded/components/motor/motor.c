#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "motor.h"


#define HIGH    1
#define LOW     0

#define IN1_PIN 27
#define IN2_PIN 26
#define IN3_PIN 25
#define IN4_PIN 33


#define MOTOR_TIME_MS 150       // How long the motor will perform each move operation

static QueueHandle_t motor_cmd_queue;

void motor_init(void) {

    /*Configure pins to output mode*/
    gpio_config_t config = {
        .pin_bit_mask = (1ULL << IN1_PIN) | (1ULL << IN2_PIN) | (1ULL << IN3_PIN) | (1ULL << IN4_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    gpio_config(&config);

    motor_stop();

    /*Create a queue for size 5 for motor commands*/
    motor_cmd_queue = xQueueCreate(5,sizeof(motor_command_t));
}

void turn_left() {
    //left
    gpio_set_level(IN1_PIN, HIGH);
    gpio_set_level(IN2_PIN, LOW);
    
    //right
    gpio_set_level(IN3_PIN, LOW);
    gpio_set_level(IN4_PIN, HIGH);
}

void turn_right() {
    //left
    gpio_set_level(IN1_PIN, LOW);
    gpio_set_level(IN2_PIN, HIGH);

    //right
    gpio_set_level(IN3_PIN, HIGH);
    gpio_set_level(IN4_PIN, LOW);
}

void move_forward() {
    gpio_set_level(IN1_PIN, LOW);
    gpio_set_level(IN2_PIN, HIGH);
    gpio_set_level(IN3_PIN, LOW);
    gpio_set_level(IN4_PIN, HIGH);
}

void move_backward() {
    gpio_set_level(IN1_PIN, HIGH);
    gpio_set_level(IN2_PIN, LOW);
    gpio_set_level(IN3_PIN, HIGH);
    gpio_set_level(IN4_PIN, LOW);
}

void motor_stop() {
    gpio_set_level(IN1_PIN, LOW);
    gpio_set_level(IN2_PIN, LOW);
    gpio_set_level(IN3_PIN, LOW);
    gpio_set_level(IN4_PIN, LOW);
}

void motor_control_task(void *arg) {
    
    motor_command_t motor_cmd;

    while (1)
    {
        if (xQueueReceive(motor_cmd_queue, &motor_cmd, portMAX_DELAY)) {
            switch (motor_cmd)
            {
                case MOTOR_CMD_STOP:
                    motor_stop();
                    break;
                
                case MOTOR_CMD_FORWARD:
                    move_forward();
                    break;
                
                case MOTOR_CMD_BACKWARD:
                    move_backward();
                    break;
                    
                case MOTOR_CMD_RIGHT:
                    turn_right();
                    break;

                case MOTOR_CMD_LEFT:
                    turn_left();
                    break;
                    
                default:
                    motor_stop();
                    break;
            }
        
        }
        vTaskDelay(pdMS_TO_TICKS(MOTOR_TIME_MS));
        motor_stop();
    }
    
}

void motor_send_cmd(motor_command_t motor_cmd) {
    xQueueSend(motor_cmd_queue, &motor_cmd, 0);
}