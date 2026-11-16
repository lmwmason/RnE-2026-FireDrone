/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "ukf_level.h"
#include "rate_core.h"
#include <math.h>
void fdLevelSelect(fdLevelState *s,const float baseline[2],const float ukf[2],bool enabled,bool permitted,bool valid,uint32_t ageUs,float dt,float out[2])
{
    out[0]=baseline[0];out[1]=baseline[1];s->active=false;s->applied[0]=s->applied[1]=0;
    if(!enabled || !permitted || s->hardFault){s->blend=0;s->recovery=0;return;}
    uint8_t hard=0;
    if(!valid || ageUs>20000)hard=1;
    if(!fdFinite(dt) || dt<=0 || dt>.002f)hard=2;
    float delta[2];
    for(int i=0;i<2;i++){
        delta[i]=ukf[i]-baseline[i];
        if(!fdFinite(baseline[i]) || !fdFinite(ukf[i]))hard=3;
    }
    if(hard){s->hardFault=true;s->rejected=true;s->reason=hard;s->blend=0;s->recovery=0;return;}
    /* Selected UKF stays selected across attitude disagreement and large tilt.
     * Only invalid/stale data, timing faults and explicit mode permission can exit.
     * Retain the existing one-second entry blend and rate-controller warmup. */
    s->rejected=false;s->reason=0;s->recovery=0;
    s->blend=fminf(1,s->blend+dt);s->active=true;
    for(int i=0;i<2;i++){s->applied[i]=s->blend*delta[i];out[i]=baseline[i]+s->applied[i];}
}
