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
    float delta[2],maxDelta=0,maxAngle=0;
    for(int i=0;i<2;i++){
        delta[i]=ukf[i]-baseline[i];
        if(!fdFinite(baseline[i]) || !fdFinite(ukf[i]))hard=3;
        maxDelta=fmaxf(maxDelta,fabsf(delta[i]));
        maxAngle=fmaxf(maxAngle,fmaxf(fabsf(ukf[i]),fabsf(baseline[i])));
    }
    if(!hard && maxDelta>20)hard=4;
    if(hard){s->hardFault=true;s->rejected=true;s->reason=hard;s->blend=0;s->recovery=0;return;}
    /* Slightly wider maneuver envelope; do not retain a transient breach forever.
     * Return only after 0.5 s within a tighter envelope, then ramp for one second.
     * Timing, invalid estimates and gross disagreement remain latched until reboot. */
    if(maxDelta>8 || maxAngle>60){
        s->rejected=true;s->reason=maxDelta>8 ? 5 : 6;s->blend=0;s->recovery=0;return;
    }
    if(s->rejected){
        if(maxDelta<=4 && maxAngle<=45)s->recovery+=dt;else s->recovery=0;
        if(s->recovery<.5f)return;
        s->rejected=false;s->reason=0;s->recovery=0;
    }
    s->blend=fminf(1,s->blend+dt);s->active=true;
    for(int i=0;i<2;i++){s->applied[i]=s->blend*delta[i];out[i]=baseline[i]+s->applied[i];}
}
