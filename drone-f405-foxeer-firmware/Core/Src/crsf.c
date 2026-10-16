#include "crsf.h"
#include "main.h"
#include <string.h>

extern UART_HandleTypeDef huart2;

#define CRSF_SYNC_BYTE      0xC8
#define CRSF_TYPE_RC_CHANNELS 0x16
#define CRSF_RX_BUF_SIZE    64
#define CRSF_NUM_CHANNELS   16

static uint8_t s_rxBuf[CRSF_RX_BUF_SIZE];
static uint16_t s_channelsRaw[CRSF_NUM_CHANNELS];
static volatile uint32_t s_lastFrameTick;

static uint8_t CRSF_Crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0xD5) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static void CRSF_UnpackChannels(const uint8_t *payload)
{
    /* 16 channels x 11 bits, little-endian packed */
    uint32_t bitBuf = 0;
    uint8_t bitCount = 0;
    uint8_t byteIdx = 0;

    for (uint8_t ch = 0; ch < CRSF_NUM_CHANNELS; ch++) {
        while (bitCount < 11) {
            bitBuf |= ((uint32_t)payload[byteIdx++]) << bitCount;
            bitCount += 8;
        }
        s_channelsRaw[ch] = (uint16_t)(bitBuf & 0x07FF);
        bitBuf >>= 11;
        bitCount -= 11;
    }
}

static void CRSF_ParseFrame(const uint8_t *buf, uint16_t size)
{
    if (size < 4 || buf[0] != CRSF_SYNC_BYTE) {
        return;
    }

    uint8_t len = buf[1];
    if (len < 2 || (uint16_t)(len + 2) > size) {
        return;
    }

    uint8_t type = buf[2];
    uint8_t crcReceived = buf[2 + len - 1];
    uint8_t crcCalc = CRSF_Crc8(&buf[2], (uint8_t)(len - 1));

    if (crcCalc != crcReceived) {
        return;
    }

    if (type == CRSF_TYPE_RC_CHANNELS) {
        CRSF_UnpackChannels(&buf[3]);
        s_lastFrameTick = HAL_GetTick();
    }
}

void CRSF_Init(void)
{
    for (uint8_t i = 0; i < CRSF_NUM_CHANNELS; i++) {
        s_channelsRaw[i] = 992; /* center */
    }
    s_lastFrameTick = 0;
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rxBuf, CRSF_RX_BUF_SIZE);
}

void CRSF_OnUartEvent(uint16_t size)
{
    CRSF_ParseFrame(s_rxBuf, size);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, s_rxBuf, CRSF_RX_BUF_SIZE);
}

bool CRSF_IsLinkUp(uint32_t now_ms)
{
    return (now_ms - s_lastFrameTick) < CRSF_FAILSAFE_TIMEOUT_MS;
}

uint16_t CRSF_GetChannelUs(uint8_t index)
{
    if (index >= CRSF_NUM_CHANNELS) {
        return 1500;
    }
    float us = 988.0f + ((float)s_channelsRaw[index] - 172.0f) * 0.62477f;
    if (us < 988.0f)  us = 988.0f;
    if (us > 2012.0f) us = 2012.0f;
    return (uint16_t)us;
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == USART2) {
        CRSF_OnUartEvent(size);
    }
}
