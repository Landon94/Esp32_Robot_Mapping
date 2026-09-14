#include "lwip/sockets.h"
#include "esp_log.h"

#define HOST_IP_ADDR ""
#define PORT 5500

static const char *TAG = "UDP";
static int sock = -1;
static struct sockaddr_in dest_addr;

void udp_init_cl(void) {
    
    dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);  // Converts ip into binary
    dest_addr.sin_family      = AF_INET;                  // IPv4
    dest_addr.sin_port        = htons(PORT);              // Converts port into network byte order
    
    sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket");
        return;
    }

    ESP_LOGI(TAG, "UDP socket created");;
}

int udp_send_msg(void *data, size_t data_length) {

    if (sock == -1) {
        ESP_LOGE(TAG, "Socket not initialized");
        return -1;
    }

    return sendto(sock, data, data_length, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
}
