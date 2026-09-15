#ifndef LIDAR_PROTOCAL_H
#define LIDAR_PROTOCAL_H

#include <stdint.h>
#include <stdbool.h>

#include "../telemetry/telemetry.h"

bool parse_byte(uint8_t byte, telemetry_lidar_t *sample);
void parse_packet(telemetry_lidar_t *sample);
bool check_head(void);

#endif
