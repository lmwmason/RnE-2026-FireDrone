#ifndef ALLOCATION_MOTOR_H
#define ALLOCATION_MOTOR_H

#include <stdint.h>

#define MOTOR_PWM_MIN_US   1000U
#define MOTOR_PWM_MAX_US   2000U
#define MOTOR_PWM_DISARM_US 1000U

void    motor_init(void);
void    motor_arm(void);
void    motor_disarm(void);
void    motor_set(uint8_t idx, float normalized);
void    motor_set_all(const float u[4]);
void    motor_set_us(uint8_t idx, uint16_t pulse_us);

#endif
