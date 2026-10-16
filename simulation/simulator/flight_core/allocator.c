#include "allocator.h"
#include <math.h>
#include <string.h>

bool fc_allocator_init(FcAllocator *a, const FcMotor motors[8], const float weights[4])
{
    if(!a) return false;
    memset(a, 0, sizeof(*a));
    if(!motors || !weights) return false;
    for (int k=0;k<4;k++) {
        if (!isfinite(weights[k]) || weights[k]<=0) return false;
        a->weight[k]=weights[k];
    }
    for (int j=0;j<8;j++) {
        const FcMotor *m=&motors[j];
        if (!isfinite(m->x) || !isfinite(m->y) || !isfinite(m->thrust) ||
            !isfinite(m->yaw_torque) || m->thrust<=0) return false;
        a->B[0][j]=m->thrust;
        a->B[1][j]=m->y*m->thrust;
        a->B[2][j]=-m->x*m->thrust;
        a->B[3][j]=m->yaw_torque;
        for(int k=0;k<4;k++) if(!isfinite(a->B[k][j])) return false;
    }
    a->initialized=true;
    return true;
}

/* Bounded weighted least squares, coordinate descent with a projected-gradient
 * stopping check. All iterates respect motor bounds; a failed motor is zero.
 * Small regularization removes singularity from redundant X8 actuators. */
bool fc_allocate(FcAllocator *a, const float request[4], uint8_t failed, FcAllocation *out)
{
    if(!out) return false;
    memset(out,0,sizeof(*out));
    if(!a || !a->initialized || !request) return false;
    float H[8][8]={{0}}, b[8]={0};
    for(int k=0;k<4;k++) if(!isfinite(request[k])) return false;
    for(int i=0;i<8;i++) {
        if(!isfinite(a->u[i]) || a->u[i]<0 || a->u[i]>1) {
            memset(out,0,sizeof(*out)); return false;
        }
        out->u[i]=(failed&(1u<<i))?0:a->u[i];
        for(int k=0;k<4;k++) b[i]+=a->B[k][i]*a->weight[k]*request[k];
        for(int j=0;j<8;j++) {
            H[i][j]=i==j?1e-5f:0;
            for(int k=0;k<4;k++) H[i][j]+=a->B[k][i]*a->weight[k]*a->B[k][j];
            if(!isfinite(H[i][j])) { memset(out,0,sizeof(*out)); return false; }
        }
        if(!isfinite(b[i])) { memset(out,0,sizeof(*out)); return false; }
    }
    for(unsigned it=0;it<128;it++) {
        for(int i=0;i<8;i++) {
            if(failed&(1u<<i)) continue;
            float g=-b[i];
            for(int j=0;j<8;j++) g+=H[i][j]*out->u[j];
            out->u[i]=fminf(1,fmaxf(0,out->u[i]-g/H[i][i]));
        }
        /* Pair exchanges resolve the redundant coaxial yaw direction quickly:
         * increase one rotor while decreasing its partner. This is another
         * exact one-dimensional minimization of the same bounded objective. */
        for(int i=0;i<8;i++) for(int j=i+1;j<8;j++) {
            if((failed&(1u<<i)) || (failed&(1u<<j))) continue;
            float denom=H[i][i]+H[j][j]-2*H[i][j];
            if(denom<1e-7f) continue;
            float g=-b[i]+b[j];
            for(int k=0;k<8;k++) g+=(H[i][k]-H[j][k])*out->u[k];
            float lo=fmaxf(-out->u[i],out->u[j]-1);
            float hi=fminf(1-out->u[i],out->u[j]);
            float delta=fminf(hi,fmaxf(lo,-g/denom));
            out->u[i]+=delta; out->u[j]-=delta;
        }
        float pg=0;
        for(int i=0;i<8;i++) {
            if(failed&(1u<<i)) continue;
            float g=-b[i];
            for(int j=0;j<8;j++) g+=H[i][j]*out->u[j];
            float next=fminf(1,fmaxf(0,out->u[i]-g/H[i][i]));
            pg=fmaxf(pg,fabsf(next-out->u[i]));
        }
        out->iterations=it+1;
        if(pg<1e-6f) { out->converged=true; break; }
    }
    for(int k=0;k<4;k++) {
        for(int j=0;j<8;j++) out->achieved[k]+=a->B[k][j]*out->u[j];
        out->residual[k]=out->achieved[k]-request[k];
    }
    memcpy(a->u,out->u,sizeof(a->u));
    return true;
}
