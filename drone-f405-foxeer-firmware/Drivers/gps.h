#ifndef DRIVERS_GPS_H
#define DRIVERS_GPS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    double  lat_deg;
    double  lon_deg;
    float   alt_m;
    float   speed_ms;
    float   track_deg;
    uint8_t fix_valid;
    uint8_t satellites;
} gps_data_t;

void gps_init(void);
void gps_rx_byte(uint8_t b);
bool gps_get_latest(gps_data_t *out);

#endif
