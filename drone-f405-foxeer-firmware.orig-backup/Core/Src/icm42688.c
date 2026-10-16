#include "icm42688.h"
#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;

#define ICM_REG_DEVICE_CONFIG   0x11
#define ICM_REG_PWR_MGMT0       0x4E
#define ICM_REG_GYRO_CONFIG0    0x4F
#define ICM_REG_ACCEL_CONFIG0   0x50
#define ICM_REG_ACCEL_DATA_X1   0x1F
#define ICM_REG_WHO_AM_I        0x75
#define ICM_WHO_AM_I_VALUE      0x47

/* +/-2000 dps -> 16.4 LSB/(deg/s); +/-16g -> 2048 LSB/g */
#define ICM_GYRO_SENSITIVITY_LSB_PER_DPS  16.4f
#define ICM_ACCEL_SENSITIVITY_LSB_PER_G   2048.0f

static void ICM_CS_Low(void)  { HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_RESET); }
static void ICM_CS_High(void) { HAL_GPIO_WritePin(GYRO_CS_GPIO_Port, GYRO_CS_Pin, GPIO_PIN_SET); }

static uint8_t ICM_ReadReg(uint8_t reg)
{
    uint8_t tx[2] = { (uint8_t)(reg | 0x80), 0x00 };
    uint8_t rx[2] = { 0 };

    ICM_CS_Low();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 10);
    ICM_CS_High();
    return rx[1];
}

static void ICM_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t tx[2] = { (uint8_t)(reg & 0x7F), value };

    ICM_CS_Low();
    HAL_SPI_Transmit(&hspi1, tx, 2, 10);
    ICM_CS_High();
}

static void ICM_ReadBurst(uint8_t startReg, uint8_t *dst, uint8_t len)
{
    uint8_t tx = (uint8_t)(startReg | 0x80);

    ICM_CS_Low();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 10);
    HAL_SPI_Receive(&hspi1, dst, len, 10);
    ICM_CS_High();
}

bool ICM42688_Init(void)
{
    ICM_CS_High();
    HAL_Delay(10);

    if (ICM_ReadReg(ICM_REG_WHO_AM_I) != ICM_WHO_AM_I_VALUE) {
        return false;
    }

    ICM_WriteReg(ICM_REG_DEVICE_CONFIG, 0x01); /* soft reset */
    HAL_Delay(2);

    if (ICM_ReadReg(ICM_REG_WHO_AM_I) != ICM_WHO_AM_I_VALUE) {
        return false;
    }

    /* +/-2000dps, 1kHz ODR */
    ICM_WriteReg(ICM_REG_GYRO_CONFIG0, 0x06);
    /* +/-16g, 1kHz ODR */
    ICM_WriteReg(ICM_REG_ACCEL_CONFIG0, 0x06);
    /* Gyro + Accel Low-Noise mode */
    ICM_WriteReg(ICM_REG_PWR_MGMT0, 0x0F);
    HAL_Delay(1);

    return true;
}

bool ICM42688_Read(icm42688_data_t *out)
{
    uint8_t raw[12];
    int16_t ax, ay, az, gx, gy, gz;

    ICM_ReadBurst(ICM_REG_ACCEL_DATA_X1, raw, 12);

    ax = (int16_t)((raw[0] << 8) | raw[1]);
    ay = (int16_t)((raw[2] << 8) | raw[3]);
    az = (int16_t)((raw[4] << 8) | raw[5]);
    gx = (int16_t)((raw[6] << 8) | raw[7]);
    gy = (int16_t)((raw[8] << 8) | raw[9]);
    gz = (int16_t)((raw[10] << 8) | raw[11]);

    out->accel_g[0] = (float)ax / ICM_ACCEL_SENSITIVITY_LSB_PER_G;
    out->accel_g[1] = (float)ay / ICM_ACCEL_SENSITIVITY_LSB_PER_G;
    out->accel_g[2] = (float)az / ICM_ACCEL_SENSITIVITY_LSB_PER_G;
    out->gyro_dps[0] = (float)gx / ICM_GYRO_SENSITIVITY_LSB_PER_DPS;
    out->gyro_dps[1] = (float)gy / ICM_GYRO_SENSITIVITY_LSB_PER_DPS;
    out->gyro_dps[2] = (float)gz / ICM_GYRO_SENSITIVITY_LSB_PER_DPS;

    return true;
}

void ICM42688_CalibrateGyroBias(float bias_dps[3], uint16_t samples)
{
    icm42688_data_t d;
    float sum[3] = { 0.0f, 0.0f, 0.0f };

    for (uint16_t i = 0; i < samples; i++) {
        ICM42688_Read(&d);
        sum[0] += d.gyro_dps[0];
        sum[1] += d.gyro_dps[1];
        sum[2] += d.gyro_dps[2];
        HAL_Delay(1);
    }

    bias_dps[0] = sum[0] / (float)samples;
    bias_dps[1] = sum[1] / (float)samples;
    bias_dps[2] = sum[2] / (float)samples;
}
