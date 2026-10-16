#ifndef X8_CONFIG_H
#define X8_CONFIG_H
#include "physics.h"
#include "../flight_core/control.h"
/* Loads an explicitly uncalibrated design study. Never implies measured hardware. */
int x8_load_config(const char *path, DroneParams *plant, FcConfig *controller);
#endif
