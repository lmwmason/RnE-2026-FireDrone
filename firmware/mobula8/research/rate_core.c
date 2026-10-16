/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "rate_core.h"
#include <math.h>
#include <string.h>

/* Betaflight uses -ffast-math. isfinite() can be optimized away there. */
bool fdFinite(float x) { uint32_t u; memcpy(&u, &x, sizeof u); return (u & 0x7f800000U) != 0x7f800000U; }
static float clip(float x, float lo, float hi) { return x < lo ? lo : x > hi ? hi : x; }
void fdReset(fdState *s) { memset(s, 0, sizeof *s); }
bool fdConfigValid(const fdConfig *c)
{
    if (c->mode > 2 || !fdFinite(c->bandwidth) || c->bandwidth < 1 || c->bandwidth > 100 ||
        !fdFinite(c->rate_gain) || c->rate_gain < 1 || c->rate_gain > 100 ||
        !fdFinite(c->motor_tau) || c->motor_tau < .001f || c->motor_tau > .2f ||
        !fdFinite(c->motor_tau_yaw) || c->motor_tau_yaw < 0 || c->motor_tau_yaw > .2f || (c->motor_tau_yaw > 0 && c->motor_tau_yaw < .001f) ||
        !fdFinite(c->derivative_hz) || c->derivative_hz < 1 || c->derivative_hz > 200 ||
        !fdFinite(c->blend) || c->blend < 0 || c->blend > 1 ||
        !fdFinite(c->correction_limit) || c->correction_limit < 0 || c->correction_limit > 100 ||
        !fdFinite(c->adaptation) || c->adaptation < 0 || c->adaptation > .01f) return false;
    for (int a=0; a<3; ++a)
        if (!fdFinite(c->effectiveness[a]) || c->effectiveness[a] <= 0 || c->effectiveness[a] > 100) return false;
    return true;
}
static bool sameConfig(const fdConfig *a,const fdConfig *b)
{
    if(a->mode!=b->mode || a->bandwidth!=b->bandwidth || a->rate_gain!=b->rate_gain ||
        a->motor_tau!=b->motor_tau || a->motor_tau_yaw!=b->motor_tau_yaw || a->derivative_hz!=b->derivative_hz || a->blend!=b->blend ||
        a->correction_limit!=b->correction_limit || a->adaptation!=b->adaptation) return false;
    for(int i=0;i<3;i++)if(a->effectiveness[i]!=b->effectiveness[i])return false;
    return true;
}
void fdStep(fdState *s, const fdConfig *c, const fdInput *in, float out[3])
{
    memcpy(out, in->baseline, sizeof in->baseline);
    if (!c->mode || !fdConfigValid(c) || !in->eligible || in->saturated || !fdFinite(in->dt) || in->dt < .0001f || in->dt > .002f) {
        fdReset(s); return;
    }
    for (int a=0; a<3; ++a) {
        if (!fdFinite(in->rate[a]) || !fdFinite(in->setpoint[a]) || !fdFinite(in->baseline[a]) ||
            fabsf(in->rate[a]) > 40 || fabsf(in->setpoint[a]) > 40) { fdReset(s); return; }
    }
    if (s->initialized && !sameConfig(&s->config,c)) fdReset(s);
    if (!s->initialized) {
        s->config=*c;
        adrc_init(&s->adrc,c->bandwidth,c->bandwidth,c->bandwidth,1,1,1,c->rate_gain,0,in->dt);
        mrac_init(&s->mrac,10,10,c->adaptation,c->adaptation,1,0,in->dt);
        for (int a=0; a<3; ++a) {
            s->filtered[a]=in->rate[a]; s->adrc.axis[a].z1=in->rate[a];
            s->inverseEffectiveness[a]=1/c->effectiveness[a];
            s->filteredLag[a]=s->lag[a]=s->applied[a]=in->baseline[a]; s->mrac.axis[a].omega_m=in->rate[a];
        }
        s->initialized=true;
    }
    if(s->cachedDt!=in->dt){
        s->cachedDt=in->dt;s->inverseDt=1/in->dt;
        s->derivativeK=in->dt/(in->dt+1/(6.283185307f*c->derivative_hz));
        for(int a=0;a<3;a++){
            const float tau=a==2 && c->motor_tau_yaw>0 ? c->motor_tau_yaw : c->motor_tau;
            s->lagK[a]=in->dt/(in->dt+tau);
        }
    }
    const float k=s->derivativeK;
    float proposed[3];
    for (int a=0; a<3; ++a) {
        const float b=c->effectiveness[a];
        const float old=s->filtered[a];
        s->filtered[a]+=k*(in->rate[a]-old);
        s->alpha[a]=(s->filtered[a]-old)*s->inverseDt;
        s->lag[a]+=s->lagK[a]*(s->applied[a]-s->lag[a]);
        // Match INDI actuator and angular-acceleration low-pass phase.
        s->filteredLag[a]+=k*(s->lag[a]-s->filteredLag[a]);
        adrc_axis_t *eso=&s->adrc.axis[a]; eso->dt=in->dt;
        mrac_axis_t *m=&s->mrac.axis[a]; m->dt=in->dt;
        const bool clipped=fabsf(s->candidate[a]-in->baseline[a])>c->correction_limit;
        const bool adapt=in->saturated==false && s->warmup>=.5f && !clipped;
        // Normalize the adaptive gradient; freeze during warmup and output constraints.
        m->gamma1=m->gamma2=adapt ? c->adaptation/(1+in->rate[a]*in->rate[a]+in->setpoint[a]*in->setpoint[a]) : 0;
        (void)mrac_axis_update(m,in->rate[a],in->setpoint[a]);
        /* Original MRAC has only lower bounds. Projection prevents real-hardware drift. */
        m->theta1_hat=clip(m->theta1_hat,.8f,1.2f); m->theta2_hat=clip(m->theta2_hat,0,.2f);
        const float reference=m->theta1_hat*in->setpoint[a]+m->theta2_hat*in->rate[a];
        const float desired=adrc_axis_update(eso,in->rate[a],reference,0,b*s->lag[a]);
        s->disturbance[a]=eso->z2;
        /* Rate-axis INDI, normalized to BF PID units. BF retains X4 mixing/DShot. */
        /* INDI already cancels the measured disturbance; do not subtract ESO disturbance twice. */
        s->candidate[a]=s->filteredLag[a]+(desired+eso->z2-s->alpha[a])*s->inverseEffectiveness[a];
        float delta=clip(s->candidate[a]-in->baseline[a],-c->correction_limit,c->correction_limit);
        if (in->saturated || s->warmup < .5f || c->mode != 2) delta=0;
        /* Bumpless return after warmup; saturation resets the infeasible actuator model. */
        const float engagement=clip((s->warmup-.5f)/.25f,0,1);
        s->correction[a]=engagement*c->blend*delta;
        proposed[a]=in->baseline[a]+s->correction[a];
        if (!fdFinite(proposed[a]) || !fdFinite(s->candidate[a]) || !fdFinite(eso->z1) ||
            !fdFinite(eso->z2) || !fdFinite(m->theta1_hat) || !fdFinite(m->theta2_hat)) { fdReset(s); return; }
    }
    s->warmup+=in->dt; s->valid=true;
    for (int a=0; a<3; ++a) { out[a]=proposed[a]; s->applied[a]=out[a]; }
}
