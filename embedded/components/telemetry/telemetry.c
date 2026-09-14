#include "udp_client.h"
#include "telemetry.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static QueueHandle_t telemetry_msg_queue;
static QueueHandle_t telemetry_lidar_msg_queue;

static QueueSetHandle_t telemetry_queue_set;

void telemetry_init(void) {
    telemetry_msg_queue = xQueueCreate(32, sizeof(telemetry_msg_t));
    telemetry_lidar_msg_queue = xQueueCreate(8 ,sizeof(telemetry_lidar_msg_t));

    // Create queue set with event size being the length of all combined queues in the set
    telemetry_queue_set = xQueueCreateSet(32 + 8);

    xQueueAddToSet(telemetry_lidar_msg_queue, telemetry_queue_set);
    xQueueAddToSet(telemetry_msg_queue, telemetry_queue_set);
}


/* Generic Function for enqueueing telemetry messages*/
void telemetry_enqueue_data(void* data, telemetry_type_t type) {

    static uint32_t lidar_sequence = 0;
    static uint32_t telemetry_sequence = 0;

    switch (type)
    {
    case TELEMETRY_LIDAR:
        telemetry_lidar_msg_t msg_lidar = *(telemetry_lidar_msg_t *)data;

        msg_lidar.header.type = TELEMETRY_LIDAR;
        msg_lidar.header.sequence = lidar_sequence++;

        xQueueSendToBack(telemetry_lidar_msg_queue, &msg_lidar, 0);
        break;
    
    case TELEMETRY_RPM:
        telemetry_msg_t msg_rpm = *(telemetry_msg_t *)data;

        msg_rpm.header.type = TELEMETRY_RPM;
        msg_rpm.header.sequence = telemetry_sequence++;

        xQueueSendToBack(telemetry_msg_queue, &msg_rpm, 0);
        break;
    
    case TELEMETRY_IMU:
        break;
    }
}

void telemetry_task(void* arg) {
    telemetry_msg_t msg;
    telemetry_lidar_msg_t msg_lidar;

    for (;;) {


        QueueSetMemberHandle_t active_queue = xQueueSelectFromSet(telemetry_queue_set, portMAX_DELAY);

        if (active_queue == telemetry_lidar_msg_queue) {

            if (xQueueReceive(telemetry_lidar_msg_queue, &msg_lidar, portMAX_DELAY) == pdPASS) {
                udp_send_msg((void*)&msg_lidar, sizeof(msg_lidar));
            }
        } else if (active_queue == telemetry_msg_queue)
        {  
            if (xQueueReceive(telemetry_msg_queue, &msg, portMAX_DELAY) == pdPASS) {
                udp_send_msg((void*)&msg, sizeof(msg));
            }
        }
    }
}