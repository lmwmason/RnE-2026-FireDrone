#ifndef DRIVERS_BARO_H
#define DRIVERS_BARO_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
} baro_data_t;

bool baro_init(void);
bool baro_read(baro_data_t *out);

#endif
