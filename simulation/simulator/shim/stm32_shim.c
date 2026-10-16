#include "stm32_shim.h"
#include <string.h>

TIM_TypeDef   sim_tim1, sim_tim6, sim_tim8;
GPIO_TypeDef  sim_gpio[8];
USART_TypeDef sim_usart2;

uint32_t sim_tick_ms;
ShimImu  sim_imu_in;
float    sim_gyro_cal_bias[3];
uint8_t *sim_uart_rx_buf;
uint16_t sim_uart_rx_size;

int HAL_Init(void)                                   { return 0; }
uint32_t HAL_GetTick(void)                           { return sim_tick_ms; }
int HAL_RCC_OscConfig(RCC_OscInitTypeDef *osc)       { (void)osc; return 0; }
int HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t l) { (void)c; (void)l; return 0; }
int HAL_UART_Init(UART_HandleTypeDef *huart)         { (void)huart; return 0; }
void HAL_UART_IRQHandler(UART_HandleTypeDef *huart)  { (void)huart; }

int HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef *huart, uint8_t *buf, uint16_t size) {
    (void)huart;
    sim_uart_rx_buf  = buf;
    sim_uart_rx_size = size;
    return 0;
}

/* ---- sensor drivers: the SPI / I2C / UART transports are replaced by the
 *      simulator, everything above them (main.c, UKF, INDI, ...) is real ---- */
#include "imu.h"
#include "baro.h"
#include "gps.h"

bool imu_init(void) { return true; }

bool imu_read(imu_data_t *out) {
    memcpy(out->gyro_dps, sim_imu_in.gyro_dps, sizeof(out->gyro_dps));
    memcpy(out->accel_g,  sim_imu_in.accel_g,  sizeof(out->accel_g));
    return true;
}

void imu_calibrate_gyro(float bias_dps[3], uint16_t samples) {
    (void)samples;
    memcpy(bias_dps, sim_gyro_cal_bias, 3 * sizeof(float));
}

float sim_baro_alt_m;

bool baro_init(void) { return true; }

bool baro_read(baro_data_t *out) {
    out->pressure_pa    = 101325.0f;
    out->temperature_c  = 20.0f;
    out->altitude_m     = sim_baro_alt_m;
    return true;
}

void gps_init(void)                 {}
void gps_rx_byte(uint8_t b)         { (void)b; }

bool gps_get_latest(gps_data_t *out) {
    memset(out, 0, sizeof(*out));
    return false;
}
