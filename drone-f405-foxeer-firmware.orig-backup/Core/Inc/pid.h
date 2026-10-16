#ifndef PID_H
#define PID_H

typedef struct {
    float kp;
    float ki;
    float kd;
    float i_limit;
    float integrator;
    float prev_measurement;
} pid_t;

void PID_Init(pid_t *pid, float kp, float ki, float kd, float i_limit);
void PID_Reset(pid_t *pid);
float PID_Update(pid_t *pid, float setpoint, float measurement, float dt);

#endif /* PID_H */
