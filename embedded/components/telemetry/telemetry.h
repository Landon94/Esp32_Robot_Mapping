#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>

#define LIDAR_BATCH_SIZE 50

typedef enum {
    TELEMETRY_LIDAR,
    TELEMETRY_RPM,
    TELEMETRY_IMU
} telemetry_type_t;

typedef struct {
    float angle_deg;
    float distance_mm;
    uint8_t quality;
    bool start_flag;
} telemetry_lidar_t;

typedef struct {
    float left_rpm;
    float right_rpm;
} telemetry_rpm_t;


typedef struct {
    telemetry_type_t type;
    uint32_t sequence;
    uint64_t timestamp_us;
} telemtry_msg_header_t;

typedef struct {
    uint16_t batch_count;
    telemtry_msg_header_t header;
    telemetry_lidar_t lidar_batch[LIDAR_BATCH_SIZE];

} telemetry_lidar_msg_t;

typedef struct {
    telemtry_msg_header_t header;

    union {
        telemetry_rpm_t rpm;
    } data;
    
} telemetry_msg_t;


void telemetry_init(void);
void telemetry_enqueue_data(void* data, telemetry_type_t type); 
void telemetry_task(void* arg);

#endif