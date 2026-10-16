#include "fault_manager.h"
#include <string.h>

void fault_init(fault_ctx_t *ctx, uint32_t comm_timeout_ms, uint32_t motor_sat_timeout_ms)
{
    memset(ctx, 0, sizeof(fault_ctx_t));
    ctx->state               = FAULT_NORMAL;
    ctx->comm_timeout_ms     = comm_timeout_ms;
    ctx->motor_sat_timeout_ms = motor_sat_timeout_ms;
    ctx->last_comm_tick_ms   = 0;
}

void fault_report_sensor(fault_ctx_t *ctx, uint8_t sensor_idx)
{
    if (sensor_idx >= 8) return;
    ctx->sensor_fault_mask |= (uint8_t)(1u << sensor_idx);
}

void fault_report_comm_rx(fault_ctx_t *ctx, uint32_t now_ms)
{
    ctx->last_comm_tick_ms = now_ms;
}

void fault_report_motor_saturated(fault_ctx_t *ctx, uint8_t motor_idx, uint32_t now_ms)
{
    if (motor_idx >= 4) return;
    if (ctx->motor_sat_start_ms[motor_idx] == 0)
        ctx->motor_sat_start_ms[motor_idx] = now_ms ? now_ms : 1u;
}

void fault_clear_motor_saturation(fault_ctx_t *ctx, uint8_t motor_idx, uint32_t now_ms)
{
    (void)now_ms;
    if (motor_idx >= 4) return;
    ctx->motor_sat_start_ms[motor_idx] = 0;
}

bool fault_motor_failed(const fault_ctx_t *ctx, uint8_t motor_idx)
{
    if (motor_idx >= 4) return false;
    return (ctx->motor_fault_mask & (1u << motor_idx)) != 0;
}

bool fault_is_recoverable(const fault_ctx_t *ctx)
{
    return ctx->state != FAULT_EMERGENCY_LAND;
}

fault_state_t fault_update(fault_ctx_t *ctx, uint32_t now_ms)
{
    if (ctx->state == FAULT_EMERGENCY_LAND)
        return FAULT_EMERGENCY_LAND;

    uint32_t comm_elapsed = now_ms - ctx->last_comm_tick_ms;
    bool comm_lost = comm_elapsed > ctx->comm_timeout_ms;

    for (uint8_t i = 0; i < 4; i++) {
        if (ctx->motor_sat_start_ms[i] != 0) {
            uint32_t sat_dur = now_ms - ctx->motor_sat_start_ms[i];
            if (sat_dur > ctx->motor_sat_timeout_ms) {
                ctx->motor_fault_mask |= (uint8_t)(1u << i);
            }
        }
    }

    bool motor_fault  = ctx->motor_fault_mask != 0;
    bool sensor_fault = ctx->sensor_fault_mask != 0;

    if (motor_fault && comm_lost) {
        ctx->state = FAULT_EMERGENCY_LAND;
    } else if (comm_lost) {
        ctx->state = FAULT_COMM_LOST;
    } else if (motor_fault) {
        ctx->state = FAULT_MOTOR_FAULT;
    } else if (sensor_fault) {
        ctx->state = FAULT_SENSOR_DEGRADED;
    } else {
        ctx->state = FAULT_NORMAL;
    }

    return ctx->state;
}
