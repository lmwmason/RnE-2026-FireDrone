/* SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once
#include <stdbool.h>
#include <stdint.h>
/* Soft envelope fallback recovers with hysteresis; invalid state stays latched. */
typedef struct {float blend,recovery;bool rejected,active,hardFault;uint8_t reason;float applied[2];} fdLevelState;
void fdLevelSelect(fdLevelState *s,const float baseline[2],const float ukf[2],bool enabled,bool permitted,bool valid,uint32_t ageUs,float dt,float out[2]);
