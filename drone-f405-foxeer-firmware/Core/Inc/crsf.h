#ifndef CRSF_H
#define CRSF_H

#include <stdbool.h>
#include <stdint.h>

#define CRSF_CH_ROLL      0
#define CRSF_CH_PITCH     1
#define CRSF_CH_THROTTLE  2
#define CRSF_CH_YAW       3
#define CRSF_CH_AUX1      4

#define CRSF_FAILSAFE_TIMEOUT_MS  300U

void CRSF_Init(void);
void CRSF_OnUartEvent(uint16_t size);
bool CRSF_IsLinkUp(uint32_t now_ms);
/* returns channel value as microsecond-equivalent, ~1000-2000 */
uint16_t CRSF_GetChannelUs(uint8_t index);

#endif /* CRSF_H */
