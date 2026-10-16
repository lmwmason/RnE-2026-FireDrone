#include "x8_config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <float.h>

int x8_load_config(const char *path, DroneParams *p, FcConfig *c)
{
    if(!path || !p || !c) return 0;
    memset(p,0,sizeof(*p)); memset(c,0,sizeof(*c));
    FILE *f=fopen(path,"r"); if(!f) { perror(path); return 0; }
    double values[19]={0}; unsigned seen=0;
    const char *keys[]={"mass","arm","ixx","iyy","izz","thrust","drag_torque",
        "motor_tau","drag_lin","drag_quad","cp_height","upper_gain","lower_gain",
        "controller_mass","controller_inertia_scale","controller_thrust","controller_motor_tau","arm_mass","arm_length"};
    char line[256];
    int lineno=0, ok=1;
    while(fgets(line,sizeof(line),f)) {
        lineno++;
        char *comment=strchr(line,'#'); if(comment) *comment=0;
        char key[80], extra; double v;
        if(sscanf(line," %79s",key)!=1) continue;
        if(sscanf(line," %79[^= ] = %lf %c",key,&v,&extra)!=2 || !isfinite(v)) { ok=0; break; }
        int index=-1;
        for(int i=0;i<19;i++) if(!strcmp(key,keys[i])) index=i;
        if(index<0 || (seen&(1u<<index))) { ok=0; break; }
        values[index]=v; seen|=1u<<index;
    }
    fclose(f);
    if(!ok || seen!=((1u<<19)-1)) { fprintf(stderr,"invalid/incomplete X8 config near line %d\n",lineno); return 0; }
    for(int i=0;i<19;i++) {
        if(values[i]<0 || (values[i]==0 && i!=8 && i!=9 && i!=10)) return 0;
    }
    if(values[17]>=values[0] || values[2]>values[3]+values[4] ||
       values[3]>values[2]+values[4] || values[4]>values[2]+values[3]) {
        fprintf(stderr,"arm mass must be below total mass; principal inertia triangle inequalities required\n");
        return 0;
    }
    /* Validate controller precision conversion before constructing the plant. */
    for(int i=0;i<19;i++) if(fabs(values[i])>FLT_MAX) return 0;
    for(int i=2;i<5;i++) if(!isfinite((float)(values[i]*values[14])) ||
                           (float)(values[i]*values[14])<=0) return 0;
    if(values[11]>1 || values[12]>1 || values[7]<0.001 || values[16]<0.001) return 0;
    p->arm_mass=values[17]; p->arm_length=values[18];
    p->motors=8; p->mass=values[0]; p->arm=values[1];
    p->Ixx=values[2]; p->Iyy=values[3]; p->Izz=values[4]; p->g=9.80665;
    p->k_thrust=values[5]; p->k_drag=values[6]; p->motor_tau=values[7];
    p->drag_lin=values[8]; p->drag_quad=values[9]; p->cp_height=values[10];
    c->mass=(float)values[13]; c->motor_tau=(float)values[16];
    for(int i=0;i<3;i++) c->inertia[i]=(float)(values[2+i]*values[14]);
    /* M1..4 upper: rear-right, front-right, rear-left, front-left.
     * M5..8 lower share those arms, with opposite spin. */
    const int x[4]={-1,1,-1,1}, y[4]={-1,-1,1,1}, spin[4]={1,-1,-1,1};
    double l=p->arm/sqrt(2.0);
    for(int j=0;j<8;j++) {
        p->rotor_x[j]=x[j%4]*l; p->rotor_y[j]=y[j%4]*l;
        p->spin[j]=spin[j%4]*(j<4?1:-1);
        p->rotor_gain[j]=values[j<4?11:12];
        c->motor[j]=(FcMotor){(float)p->rotor_x[j],(float)p->rotor_y[j],
            (float)values[15], (float)(-p->spin[j]*values[6])};
    }
    return 1;
}
