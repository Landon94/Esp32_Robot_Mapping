/*
RPLIDAR A1 protocal infomation found here
https://gargantua.ai/wp-content/uploads/2024/11/slamtec-triangulation-lidar-rplidar-a1-protocol-download.pdf
*/

#include "lidar_protocal.h"

#define BYTE_1 0
#define BYTE_2 1
#define BYTE_3 2
#define BYTE_4 3
#define BYTE_5 4

static uint8_t parser_index = BYTE_1;
static uint8_t packet[5];


bool parse_byte(uint8_t byte, telemetry_lidar_t *sample) {

    switch (parser_index)
    {
    case BYTE_1:
        packet[BYTE_1] = byte;
        parser_index = BYTE_2;
        break;
    
    case BYTE_2:
        packet[BYTE_2] = byte;

        if (!check_head()) {
            // byte 2 could potentially be head so move it down to byte 1
            packet[BYTE_1] = packet[BYTE_2];
            parser_index = BYTE_2;
        } else {
            parser_index = BYTE_3;
        }
        break;
    
    case BYTE_3:
        packet[BYTE_3] = byte;
        parser_index = BYTE_4;
        break;
    
    case BYTE_4:
        packet[BYTE_4] = byte;
        parser_index = BYTE_5;
        break;
    
    case BYTE_5:
        packet[BYTE_5] = byte;
        parser_index = BYTE_1;

        parse_packet(sample);
        return true;
    }

    return false;

}

void parse_packet(telemetry_lidar_t *sample) {

    sample->start_flag = packet[BYTE_1] & 1;
    sample->quality = packet[BYTE_1] >> 2;

    uint16_t angle_q6 = ((uint16_t)packet[BYTE_3] << 7) | (packet[BYTE_2] >> 1);
    uint16_t distance_q2 = ((uint16_t)packet[BYTE_5] << 8) | packet[BYTE_4];

    //convert from fixed to floating point

    sample->angle_deg = angle_q6 / 64.0f;
    sample->distance_mm = distance_q2 / 4.0f;
}



/* 
When the start flag is set to 1 it indicates a new scan.
inv_start_flg used as a check bit and expected to be inv of start flag
check bit expected to be 1
*/
bool check_head(void) {
    uint8_t start_flg = packet[BYTE_1] & 1;
    uint8_t inv_start_flg = (packet[BYTE_1] >> 1) & 1;
    uint8_t check_bit = packet[BYTE_2] & 1;

    return (start_flg != inv_start_flg) && check_bit == 1;
}
