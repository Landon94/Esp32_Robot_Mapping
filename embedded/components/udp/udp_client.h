#ifndef UDP_CLIENT_H
#define UDP_CLIENT_H

#include <stddef.h>

void udp_init_cl(void);
int udp_send_msg(void *data, size_t data_length);

#endif