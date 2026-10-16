#include "baro.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include <math.h>
#include <stdint.h>

/* DPS310 on I2C1  SCL=PB6(AF4)  SDA=PB7(AF4)
 * I2C address: 0x77 (SDO=high) */

#define DPS_ADDR     0x77U

#define DPS_REG_PSR_B2    0x00U
#define DPS_REG_TMP_B2    0x03U
#define DPS_REG_PRS_CFG   0x06U
#define DPS_REG_TMP_CFG   0x07U
#define DPS_REG_MEAS_CFG  0x08U
#define DPS_REG_CFG_REG   0x09U
#define DPS_REG_RESET     0x0CU
#define DPS_REG_PROD_ID   0x0DU
#define DPS_REG_COEF      0x13U
#define DPS_REG_COEF_SRCE 0x28U

#define DPS_PROD_ID_VAL   0x10U

/* kT and kP scale factors for OSR=1 (single measurement) */
#define DPS_KT  524288.0f
#define DPS_KP  524288.0f

#define SEA_LEVEL_PA  101325.0f

static int32_t s_c0, s_c1;
static int32_t s_c00, s_c10, s_c01, s_c11, s_c20, s_c21, s_c30;
static uint8_t s_tmp_coef_src;

static void delay_ms(uint32_t ms)
{
    ms *= (168u / 4u) * 1000u;
    while (ms--) __NOP();
}

static int32_t sign_extend(uint32_t val, uint8_t bits)
{
    if (val & (1u << (bits - 1u))) val |= (~0u << bits);
    return (int32_t)val;
}

static void gpio_i2c_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);

    LL_GPIO_InitTypeDef g = {
        .Pin        = LL_GPIO_PIN_6 | LL_GPIO_PIN_7,
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_OPENDRAIN,
        .Pull       = LL_GPIO_PULL_NO,
        .Alternate  = LL_GPIO_AF_4,
    };
    LL_GPIO_Init(GPIOB, &g);
}

static void i2c1_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    LL_I2C_InitTypeDef i = {
        .ClockSpeed      = 100000,
        .DutyCycle       = LL_I2C_DUTYCYCLE_2,
        .OwnAddress1     = 0,
        .TypeAcknowledge = LL_I2C_ACK,
        .OwnAddrSize     = LL_I2C_OWNADDRESS1_7BIT,
    };
    LL_I2C_Init(I2C1, &i);
    LL_I2C_Enable(I2C1);
}

static bool i2c_write_reg(uint8_t addr, uint8_t reg, uint8_t val)
{
    uint32_t timeout = 10000;

    while (LL_I2C_IsActiveFlag_BUSY(I2C1) && --timeout);
    if (!timeout) return false;

    LL_I2C_GenerateStartCondition(I2C1);
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_SB(I2C1) && --timeout);
    if (!timeout) return false;

    LL_I2C_TransmitData8(I2C1, (uint8_t)(addr << 1));
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1) && --timeout);
    if (!timeout) { LL_I2C_GenerateStopCondition(I2C1); return false; }
    LL_I2C_ClearFlag_ADDR(I2C1);

    LL_I2C_TransmitData8(I2C1, reg);
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_TXE(I2C1) && --timeout);
    if (!timeout) { LL_I2C_GenerateStopCondition(I2C1); return false; }

    LL_I2C_TransmitData8(I2C1, val);
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_BTF(I2C1) && --timeout);

    LL_I2C_GenerateStopCondition(I2C1);
    return timeout > 0;
}

static bool i2c_read_regs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n)
{
    uint32_t timeout = 10000;

    while (LL_I2C_IsActiveFlag_BUSY(I2C1) && --timeout);
    if (!timeout) return false;

    LL_I2C_GenerateStartCondition(I2C1);
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_SB(I2C1) && --timeout);
    if (!timeout) return false;

    LL_I2C_TransmitData8(I2C1, (uint8_t)(addr << 1));
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1) && --timeout);
    if (!timeout) { LL_I2C_GenerateStopCondition(I2C1); return false; }
    LL_I2C_ClearFlag_ADDR(I2C1);

    LL_I2C_TransmitData8(I2C1, reg);
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_BTF(I2C1) && --timeout);
    if (!timeout) { LL_I2C_GenerateStopCondition(I2C1); return false; }

    LL_I2C_GenerateStartCondition(I2C1);
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_SB(I2C1) && --timeout);
    if (!timeout) return false;

    LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_ACK);
    LL_I2C_TransmitData8(I2C1, (uint8_t)((addr << 1) | 1u));
    timeout = 10000;
    while (!LL_I2C_IsActiveFlag_ADDR(I2C1) && --timeout);
    if (!timeout) { LL_I2C_GenerateStopCondition(I2C1); return false; }

    if (n == 1) LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
    LL_I2C_ClearFlag_ADDR(I2C1);

    for (uint8_t i = 0; i < n; i++) {
        if (i == n - 1) {
            LL_I2C_AcknowledgeNextData(I2C1, LL_I2C_NACK);
            LL_I2C_GenerateStopCondition(I2C1);
        }
        timeout = 10000;
        while (!LL_I2C_IsActiveFlag_RXNE(I2C1) && --timeout);
        if (!timeout) return false;
        buf[i] = LL_I2C_ReceiveData8(I2C1);
    }
    return true;
}

static bool load_coefficients(void)
{
    uint8_t c[18];
    if (!i2c_read_regs(DPS_ADDR, DPS_REG_COEF, c, 18)) return false;

    s_c0  = sign_extend(((uint32_t)c[0] << 4) | ((uint32_t)c[1] >> 4), 12);
    s_c1  = sign_extend(((uint32_t)(c[1] & 0x0Fu) << 8) | c[2], 12);
    s_c00 = sign_extend(((uint32_t)c[3] << 12) | ((uint32_t)c[4] << 4)
                       | ((uint32_t)c[5] >> 4), 20);
    s_c10 = sign_extend(((uint32_t)(c[5] & 0x0Fu) << 16) | ((uint32_t)c[6] << 8)
                       | c[7], 20);
    s_c01 = sign_extend(((uint32_t)c[8]  << 8) | c[9],  16);
    s_c11 = sign_extend(((uint32_t)c[10] << 8) | c[11], 16);
    s_c20 = sign_extend(((uint32_t)c[12] << 8) | c[13], 16);
    s_c21 = sign_extend(((uint32_t)c[14] << 8) | c[15], 16);
    s_c30 = sign_extend(((uint32_t)c[16] << 8) | c[17], 16);

    uint8_t src;
    if (!i2c_read_regs(DPS_ADDR, DPS_REG_COEF_SRCE, &src, 1)) return false;
    s_tmp_coef_src = (src >> 7) & 1u;

    return true;
}

bool baro_init(void)
{
    gpio_i2c_init();
    i2c1_init();
    delay_ms(50);

    i2c_write_reg(DPS_ADDR, DPS_REG_RESET, 0x09u);
    delay_ms(40);

    uint8_t id;
    if (!i2c_read_regs(DPS_ADDR, DPS_REG_PROD_ID, &id, 1)) return false;
    if ((id & 0x0Fu) != (DPS_PROD_ID_VAL & 0x0Fu)) return false;

    if (!load_coefficients()) return false;

    i2c_write_reg(DPS_ADDR, DPS_REG_PRS_CFG, 0x00u);
    i2c_write_reg(DPS_ADDR, DPS_REG_TMP_CFG,
                  (uint8_t)(0x00u | (s_tmp_coef_src ? 0x80u : 0x00u)));
    i2c_write_reg(DPS_ADDR, DPS_REG_CFG_REG, 0x00u);
    i2c_write_reg(DPS_ADDR, DPS_REG_MEAS_CFG, 0x07u);

    delay_ms(40);
    return true;
}

bool baro_read(baro_data_t *out)
{
    uint8_t buf[6];
    if (!i2c_read_regs(DPS_ADDR, DPS_REG_PSR_B2, buf, 6)) return false;

    int32_t praw = sign_extend(((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8)
                               | buf[2], 24);
    int32_t traw = sign_extend(((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8)
                               | buf[5], 24);

    float tsc = (float)traw / DPS_KT;
    float psc = (float)praw / DPS_KP;

    float tcomp = (float)s_c0 * 0.5f + (float)s_c1 * tsc;

    float pcomp = (float)s_c00
                + psc * ((float)s_c10 + psc * ((float)s_c20 + psc * (float)s_c30))
                + tsc * (float)s_c01
                + tsc * psc * ((float)s_c11 + psc * (float)s_c21);

    out->pressure_pa   = pcomp;
    out->temperature_c = tcomp;
    out->altitude_m    = 44330.0f * (1.0f - powf(pcomp / SEA_LEVEL_PA, 0.1903f));

    return true;
}
