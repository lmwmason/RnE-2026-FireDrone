/* Each bench_* function runs one firmware stage once, on the real source files.
 * measure_cycles.py executes them inside a Cortex-M4 emulator and counts cycles. */
#include <stdint.h>
#include <string.h>

#include "ukf.h"
#include "indi.h"
#include "adrc.h"
#include "mrac.h"
#include "cam.h"
#include "fault_manager.h"

extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;

static ukf_t       g_ukf;
static indi_t      g_indi;
static adrc_t      g_adrc;
static mrac_t      g_mrac;
static cam_t       g_cam;
static fault_ctx_t g_fault;

static float g_gyro[3]  = { 0.02f, -0.01f, 0.005f };
static float g_accel[3] = { 0.1f, -0.2f, 9.7f };
static float g_ref[3]   = { 1.0f, -0.5f, 0.2f };
static float g_alpha[3] = { 30.0f, -20.0f, 5.0f };
static float g_u[4];
static volatile float g_sink;

/* return target for the emulator: every bench_* function simply returns here */
__attribute__((noinline)) void bench_ret(void) { for (;;) { } }

void bench_boot(void)
{
    for (uint32_t *s = &_sidata, *d = &_sdata; d < &_edata;) *d++ = *s++;
    for (uint32_t *d = &_sbss; d < &_ebss;) *d++ = 0;
}

void bench_init(void)
{
    ukf_init(&g_ukf, 0.5f, 2.0f, 0.0f);
    indi_init(&g_indi, 0.080f, 5.0f, 0.020f, 0.003f, 0.003f, 0.005f, 60.0f, 0.001f);
    adrc_init(&g_adrc, 20.0f, 20.0f, 15.0f, 94.3f, 94.3f, 4.0f, 10.0f, 2.0f, 0.001f);
    mrac_init(&g_mrac, 10.0f, 10.0f, 0.001f, 0.001f, 1.0f, 0.1f, 0.001f);
    cam_init(&g_cam, 5.0f, 0.02f, 0.080f);
    fault_init(&g_fault, 300, 2000);
}

/* worst case: gate disabled so the whole update (K, P) is executed */
void bench_ukf_predict(void) { ukf_predict(&g_ukf, g_gyro, 0.001f); }
void bench_ukf_update(void)  { ukf_update(&g_ukf, g_accel, g_gyro, 0.0f); }

void bench_indi(void)  { indi_update(&g_indi, g_gyro, g_alpha, 0.3f, g_u); g_sink = g_u[0]; }
void bench_adrc(void)  { float o[3]; adrc_update(&g_adrc, g_gyro, g_ref, g_ref, g_alpha, o); g_sink = o[0]; }
void bench_mrac(void)  { float o[3]; mrac_update(&g_mrac, g_gyro, g_ref, o); g_sink = o[0]; }

void bench_cam(void)
{
    float tau[4] = { 6.0f, 0.01f, -0.01f, 0.002f };
    cam_set_fault(&g_cam, 1);
    cam_solve(&g_cam, tau, g_u);
    cam_clear_faults(&g_cam);
}

void bench_fault(void) { g_sink = (float)fault_update(&g_fault, 1000); }
