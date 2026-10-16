#include "fault_manager.h"
#include <string.h>
#include <math.h>

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

void fault_report_motor_failed(fault_ctx_t *ctx, uint8_t motor_idx)
{
    if (motor_idx >= 4) return;
    ctx->motor_fault_mask |= (uint8_t)(1u << motor_idx);
}

#define DIAG_MOTOR_TAU_S   0.025f   /* assumed ESC+motor lag, measure on the rig */
#define DIAG_R_TAU_S       0.060f
#define DIAG_MIN_CMD       0.22f    /* below this a gust or arm torque can mimic a missing motor */
#define DIAG_COS_MIN       0.85f    /* roll/pitch/yaw pattern vs the G column */
#define DIAG_AGREE_LO      0.6f     /* yaw-based vs roll/pitch-based missing command */
#define DIAG_AGREE_HI      1.7f
#define DIAG_MISS_RATIO    0.7f     /* missing command / lagged command */
#define DIAG_HOLD_TICKS    60U
#define DIAG_BASE_TAU_S    1.5f     /* wind and arm torques have no yaw part, a damaged prop does: learn it */

int fault_motor_diag(fault_ctx_t *ctx, const float G[12], const float alpha_meas[3],
                     const float u_cmd[4], float dt)
{
    float ka = dt / DIAG_MOTOR_TAU_S;
    float kr = dt / DIAG_R_TAU_S;
    float pred[3] = { 0.0f, 0.0f, 0.0f };
    for (int j = 0; j < 4; j++) {
        ctx->diag_u_lp[j] += ka * (u_cmd[j] - ctx->diag_u_lp[j]);
        for (int i = 0; i < 3; i++) pred[i] += G[i * 4 + j] * ctx->diag_u_lp[j];
    }
    for (int i = 0; i < 3; i++)
        ctx->diag_r_lp[i] += kr * ((alpha_meas[i] - pred[i]) - ctx->diag_r_lp[i]);
    if (!ctx->diag_init) {
        ctx->diag_yaw_base = ctx->diag_r_lp[2];
        ctx->diag_init = 1;
    }
    ctx->diag_yaw_base += (dt / DIAG_BASE_TAU_S) * (ctx->diag_r_lp[2] - ctx->diag_yaw_base);
    const float r[3] = { ctx->diag_r_lp[0], ctx->diag_r_lp[1], ctx->diag_r_lp[2] - ctx->diag_yaw_base };

    int found = -1;
    for (int j = 0; j < 4; j++) {
        float gx = G[j], gy = G[4 + j], gz = G[8 + j];
        float n2  = gx * gx + gy * gy + gz * gz;
        float dot = -(gx * r[0] + gy * r[1] + gz * r[2]);                   /* r = -G_j * missing */
        float rn2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2];
        float miss_rp  = -(gx * r[0] + gy * r[1]) / (gx * gx + gy * gy);    /* roll + pitch */
        float miss_yaw = -(gz * r[2]) / (gz * gz);                          /* yaw */

        bool bad = ctx->diag_u_lp[j] > DIAG_MIN_CMD
                && dot > DIAG_COS_MIN * sqrtf(n2 * rn2 + 1e-9f)
                && miss_rp > DIAG_MISS_RATIO * ctx->diag_u_lp[j]
                && miss_yaw > DIAG_AGREE_LO * miss_rp && miss_yaw < DIAG_AGREE_HI * miss_rp;
        ctx->diag_cnt[j] = bad ? (uint16_t)(ctx->diag_cnt[j] + 1u) : 0u;
        if (ctx->diag_cnt[j] >= DIAG_HOLD_TICKS && !(ctx->motor_fault_mask & (1u << j))) {
            fault_report_motor_failed(ctx, (uint8_t)j);
            found = j;
        }
    }
    return found;
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
