#include "imu.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include <stdint.h>

/* ICM-42688-P on SPI1
 * SCK=PA5 MISO=PA6 MOSI=PA7 CS=PA4  (AF5 for SPI pins) */

#define ICM_SPI        SPI1
#define ICM_CS_PORT    GPIOA
#define ICM_CS_PIN     LL_GPIO_PIN_4

#define ICM_REG_WHO_AM_I     0x75U
#define ICM_REG_DEVICE_CFG   0x11U
#define ICM_REG_PWR_MGMT0    0x4EU
#define ICM_REG_GYRO_CFG0    0x4FU
#define ICM_REG_ACCEL_CFG0   0x50U
#define ICM_REG_ACCEL_DATA   0x1FU

#define ICM_WHO_AM_I_VAL     0x47U

#define GYRO_SCALE   (1.0f / 16.4f)
#define ACCEL_SCALE  (1.0f / 2048.0f)

static inline void cs_low(void)  { LL_GPIO_ResetOutputPin(ICM_CS_PORT, ICM_CS_PIN); }
static inline void cs_high(void) { LL_GPIO_SetOutputPin(ICM_CS_PORT, ICM_CS_PIN); }

static uint8_t spi_rw(uint8_t tx)
{
    while (!LL_SPI_IsActiveFlag_TXE(ICM_SPI));
    LL_SPI_TransmitData8(ICM_SPI, tx);
    while (!LL_SPI_IsActiveFlag_RXNE(ICM_SPI));
    return LL_SPI_ReceiveData8(ICM_SPI);
}

static void reg_write(uint8_t addr, uint8_t val)
{
    cs_low();
    spi_rw(addr & 0x7Fu);
    spi_rw(val);
    cs_high();
}

static uint8_t reg_read(uint8_t addr)
{
    cs_low();
    spi_rw(addr | 0x80u);
    uint8_t v = spi_rw(0x00u);
    cs_high();
    return v;
}

static void regs_read(uint8_t addr, uint8_t *buf, uint8_t n)
{
    cs_low();
    spi_rw(addr | 0x80u);
    for (uint8_t i = 0; i < n; i++) buf[i] = spi_rw(0x00u);
    cs_high();
}

static void delay_us(uint32_t us)
{
    us *= (168u / 4u);
    while (us--) __NOP();
}

static void gpio_spi_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef g = {
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_VERY_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_NO,
        .Alternate  = LL_GPIO_AF_5,
    };
    g.Pin = LL_GPIO_PIN_5 | LL_GPIO_PIN_6 | LL_GPIO_PIN_7;
    LL_GPIO_Init(GPIOA, &g);

    g.Mode = LL_GPIO_MODE_OUTPUT;
    g.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    g.Alternate = 0;
    g.Pin = ICM_CS_PIN;
    LL_GPIO_Init(GPIOA, &g);
    cs_high();
}

static void spi1_init(void)
{
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);

    LL_SPI_InitTypeDef s = {
        .TransferDirection = LL_SPI_FULL_DUPLEX,
        .Mode              = LL_SPI_MODE_MASTER,
        .DataWidth         = LL_SPI_DATAWIDTH_8BIT,
        .ClockPolarity     = LL_SPI_POLARITY_LOW,
        .ClockPhase        = LL_SPI_PHASE_1EDGE,
        .NSS               = LL_SPI_NSS_SOFT,
        .BaudRate          = LL_SPI_BAUDRATEPRESCALER_DIV8,
        .BitOrder          = LL_SPI_MSB_FIRST,
        .CRCCalculation    = LL_SPI_CRCCALCULATION_DISABLE,
    };
    LL_SPI_Init(SPI1, &s);
    LL_SPI_SetStandard(SPI1, LL_SPI_PROTOCOL_MOTOROLA);
    LL_SPI_Enable(SPI1);
}

bool imu_init(void)
{
    gpio_spi_init();
    spi1_init();

    delay_us(1000);

    reg_write(ICM_REG_DEVICE_CFG, 0x01u);
    delay_us(2000);

    if (reg_read(ICM_REG_WHO_AM_I) != ICM_WHO_AM_I_VAL) return false;

    reg_write(ICM_REG_PWR_MGMT0,  0x0Fu);
    delay_us(300);
    reg_write(ICM_REG_GYRO_CFG0,  0x06u);
    reg_write(ICM_REG_ACCEL_CFG0, 0x06u);
    delay_us(300);

    return true;
}

bool imu_read(imu_data_t *out)
{
    uint8_t buf[12];
    regs_read(ICM_REG_ACCEL_DATA, buf, 12);

    int16_t ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4]  << 8) | buf[5]);
    int16_t gx = (int16_t)((buf[6]  << 8) | buf[7]);
    int16_t gy = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);

    out->accel_g[0] = ax * ACCEL_SCALE;
    out->accel_g[1] = ay * ACCEL_SCALE;
    out->accel_g[2] = az * ACCEL_SCALE;

    out->gyro_dps[0] = gx * GYRO_SCALE;
    out->gyro_dps[1] = gy * GYRO_SCALE;
    out->gyro_dps[2] = gz * GYRO_SCALE;

    return true;
}

void imu_calibrate_gyro(float bias_dps[3], uint16_t samples)
{
    float sum[3] = {0};
    for (uint16_t i = 0; i < samples; i++) {
        imu_data_t d;
        imu_read(&d);
        sum[0] += d.gyro_dps[0];
        sum[1] += d.gyro_dps[1];
        sum[2] += d.gyro_dps[2];
        delay_us(1000);
    }
    float inv_n = 1.0f / samples;
    bias_dps[0] = sum[0] * inv_n;
    bias_dps[1] = sum[1] * inv_n;
    bias_dps[2] = sum[2] * inv_n;
}
