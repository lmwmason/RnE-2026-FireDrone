#include "gps.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx.h"
#include <string.h>
#include <stdlib.h>

/* GPS on UART5  TX=PC12(AF8)  RX=PD2(AF8)  9600 baud */

#define NMEA_MAX_LEN 96U

static char     s_sentence[NMEA_MAX_LEN];
static uint8_t  s_idx;
static gps_data_t s_latest;
static uint8_t  s_has_fix;

static void gpio_uart5_init(void)
{
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC
                            | LL_AHB1_GRP1_PERIPH_GPIOD);

    LL_GPIO_InitTypeDef g = {
        .Mode       = LL_GPIO_MODE_ALTERNATE,
        .Speed      = LL_GPIO_SPEED_FREQ_HIGH,
        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
        .Pull       = LL_GPIO_PULL_UP,
        .Alternate  = LL_GPIO_AF_8,
    };
    g.Pin = LL_GPIO_PIN_12;
    LL_GPIO_Init(GPIOC, &g);

    g.Pin = LL_GPIO_PIN_2;
    LL_GPIO_Init(GPIOD, &g);
}

static void uart5_ll_init(void)
{
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_UART5);

    LL_USART_InitTypeDef u = {
        .BaudRate            = 9600,
        .DataWidth           = LL_USART_DATAWIDTH_8B,
        .StopBits            = LL_USART_STOPBITS_1,
        .Parity              = LL_USART_PARITY_NONE,
        .TransferDirection   = LL_USART_DIRECTION_TX_RX,
        .HardwareFlowControl = LL_USART_HWCONTROL_NONE,
        .OverSampling        = LL_USART_OVERSAMPLING_16,
    };
    LL_USART_Init(UART5, &u);
    LL_USART_EnableIT_RXNE(UART5);
    LL_USART_Enable(UART5);

    NVIC_SetPriority(UART5_IRQn, 6);
    NVIC_EnableIRQ(UART5_IRQn);
}

static double nmea_coord(const char *field, char hemi)
{
    if (!field || field[0] == '\0') return 0.0;
    double raw = atof(field);
    int deg = (int)(raw / 100.0);
    double min = raw - deg * 100.0;
    double val = deg + min / 60.0;
    if (hemi == 'S' || hemi == 'W') val = -val;
    return val;
}

static uint8_t nmea_checksum(const char *s)
{
    uint8_t cs = 0;
    while (*s && *s != '*') cs ^= (uint8_t)*s++;
    return cs;
}

static void parse_rmc(char *buf)
{
    char *tok[12];
    uint8_t n = 0;
    char *p = buf;
    while (n < 12) {
        tok[n++] = p;
        char *c = p;
        while (*c && *c != ',') c++;
        if (*c == ',') { *c = '\0'; p = c + 1; } else break;
    }
    if (n < 9) return;

    if (tok[1][0] != 'A') { s_latest.fix_valid = 0; return; }

    s_latest.lat_deg   = nmea_coord(tok[2], tok[3][0]);
    s_latest.lon_deg   = nmea_coord(tok[4], tok[5][0]);
    s_latest.speed_ms  = (float)atof(tok[6]) * 0.514444f;
    s_latest.track_deg = (float)atof(tok[7]);
    s_latest.fix_valid = 1;
    s_has_fix = 1;
}

static void parse_gga(char *buf)
{
    char *tok[15];
    uint8_t n = 0;
    char *p = buf;
    while (n < 15) {
        tok[n++] = p;
        char *c = p;
        while (*c && *c != ',') c++;
        if (*c == ',') { *c = '\0'; p = c + 1; } else break;
    }
    if (n < 10) return;
    if (tok[5][0] == '0') return;
    s_latest.alt_m      = (float)atof(tok[8]);
    s_latest.satellites = (uint8_t)atoi(tok[6]);
}

static void process_sentence(void)
{
    if (s_sentence[0] != '$') return;
    char *body = s_sentence + 1;

    char *star = body;
    while (*star && *star != '*') star++;
    if (*star != '*') return;

    uint8_t given = (uint8_t)strtol(star + 1, NULL, 16);
    *star = '\0';
    if (nmea_checksum(body) != given) return;

    if (strncmp(body, "GNRMC,", 6) == 0 || strncmp(body, "GPRMC,", 6) == 0)
        parse_rmc(body + 6);
    else if (strncmp(body, "GNGGA,", 6) == 0 || strncmp(body, "GPGGA,", 6) == 0)
        parse_gga(body + 6);
}

void gps_init(void)
{
    memset(&s_latest, 0, sizeof(s_latest));
    s_idx = 0;
    s_has_fix = 0;
    gpio_uart5_init();
    uart5_ll_init();
}

void gps_rx_byte(uint8_t b)
{
    if (b == '$') {
        s_idx = 0;
    }
    if (s_idx < NMEA_MAX_LEN - 1) {
        s_sentence[s_idx++] = (char)b;
        s_sentence[s_idx]   = '\0';
    }
    if (b == '\n') {
        process_sentence();
        s_idx = 0;
    }
}

bool gps_get_latest(gps_data_t *out)
{
    if (!s_has_fix) return false;
    *out = s_latest;
    return true;
}

void UART5_IRQHandler(void)
{
    if (LL_USART_IsActiveFlag_RXNE(UART5))
        gps_rx_byte(LL_USART_ReceiveData8(UART5));
}
