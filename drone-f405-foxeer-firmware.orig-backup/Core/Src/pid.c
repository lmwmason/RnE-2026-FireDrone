#include "pid.h"

void PID_Init(pid_t *pid, float kp, float ki, float kd, float i_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
}

void PID_Reset(pid_t *pid)
{
    pid->integrator = 0.0f;
    pid->prev_measurement = 0.0f;
}

float PID_Update(pid_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    pid->integrator += error * dt;
    if (pid->integrator > pid->i_limit)  pid->integrator = pid->i_limit;
    if (pid->integrator < -pid->i_limit) pid->integrator = -pid->i_limit;

    /* derivative on measurement avoids derivative kick from stick steps */
    float derivative = -(measurement - pid->prev_measurement) / dt;
    pid->prev_measurement = measurement;

    return pid->kp * error + pid->ki * pid->integrator + pid->kd * derivative;
}
