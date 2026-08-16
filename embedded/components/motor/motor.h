#ifndef MOTOR_H
#define MOTOR_H

typedef enum {
    MOTOR_CMD_STOP,
    MOTOR_CMD_FORWARD,
    MOTOR_CMD_BACKWARD,
    MOTOR_CMD_RIGHT,
    MOTOR_CMD_LEFT,
} motor_command_t;


void motor_init(void);
void turn_left(void);
void turn_right(void);
void move_forward(void);
void move_backward(void);
void motor_stop(void);
void motor_send_cmd(motor_command_t motor_cmd);
void motor_control_task(void *arg);

#endif