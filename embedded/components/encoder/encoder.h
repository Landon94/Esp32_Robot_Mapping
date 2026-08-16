#ifndef ENCODER_H
#define ENCODER_H

extern float g_rpm_1;
extern float g_rpm_2;

void rpm_init(void);
void rpm_task(void *arg);

#endif