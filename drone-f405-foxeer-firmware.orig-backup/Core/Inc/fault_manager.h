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
} fault_ctx_t;

void fault_init(fault_ctx_t *ctx, uint32_t comm_timeout_ms, uint32_t motor_sat_timeout_ms);
void fault_report_sensor(fault_ctx_t *ctx, uint8_t sensor_idx);
void fault_report_comm_rx(fault_ctx_t *ctx, uint32_t now_ms);
void fault_report_motor_saturated(fault_ctx_t *ctx, uint8_t motor_idx, uint32_t now_ms);
void fault_clear_motor_saturation(fault_ctx_t *ctx, uint8_t motor_idx, uint32_t now_ms);
fault_state_t fault_update(fault_ctx_t *ctx, uint32_t now_ms);
bool fault_motor_failed(const fault_ctx_t *ctx, uint8_t motor_idx);
bool fault_is_recoverable(const fault_ctx_t *ctx);

#endif
