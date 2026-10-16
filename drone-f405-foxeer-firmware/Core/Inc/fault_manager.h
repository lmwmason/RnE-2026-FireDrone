#ifndef CORE_FAULT_MANAGER_H
#define CORE_FAULT_MANAGER_H

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    FAULT_NORMAL = 0,
    FAULT_SENSOR_DEGRADED,
    FAULT_MOTOR_FAULT,
    FAULT_COMM_LOST,
    FAULT_EMERGENCY_LAND
} fault_state_t;

typedef struct {
    fault_state_t state;
    uint8_t       sensor_fault_mask;
    uint8_t       motor_fault_mask;
    uint32_t      last_comm_tick_ms;
    uint32_t      comm_timeout_ms;
    uint32_t      motor_sat_start_ms[4];
    uint32_t      motor_sat_timeout_ms;
    float         diag_u_lp[4];     /* motor-command model (first-order lag) */
    float         diag_r_lp[3];     /* low-passed torque residual, rad/s^2 */
    float         diag_yaw_base;    /* slow yaw residual: propeller damage already on board */
    uint8_t       diag_init;
    uint16_t      diag_cnt[4];
} fault_ctx_t;

void fault_init(fault_ctx_t *ctx, uint32_t comm_timeout_ms, uint32_t motor_sat_timeout_ms);
void fault_report_sensor(fault_ctx_t *ctx, uint8_t sensor_idx);
void fault_report_comm_rx(fault_ctx_t *ctx, uint32_t now_ms);
void fault_report_motor_saturated(fault_ctx_t *ctx, uint8_t motor_idx, uint32_t now_ms);
void fault_clear_motor_saturation(fault_ctx_t *ctx, uint8_t motor_idx, uint32_t now_ms);
void fault_report_motor_failed(fault_ctx_t *ctx, uint8_t motor_idx);
/* Residual isolation.  alpha_meas - G*u(lagged) is the disturbance torque.  A dead motor i gives
 * -G[:,i]*u_i: a roll/pitch pattern AND a yaw part.  Wind and arm torque have no yaw part, so the
 * two independent estimates of the missing command (roll/pitch vs yaw) must agree.
 * Returns the index of a newly detected dead motor, or -1. */
int  fault_motor_diag(fault_ctx_t *ctx, const float G[12], const float alpha_meas[3],
                      const float u_cmd[4], float dt);
fault_state_t fault_update(fault_ctx_t *ctx, uint32_t now_ms);
bool fault_motor_failed(const fault_ctx_t *ctx, uint8_t motor_idx);
bool fault_is_recoverable(const fault_ctx_t *ctx);

#endif
