/* The bridge internals are included only in this test to inspect sensor
 * isolation and fault timing without adding a truth/debug API to the controller. */
#include "../src/live_sim.c"
#include <assert.h>
#include <stdio.h>

int main(void)
{
    LiveSim *s=live_create("configs/x8_design_study.conf",42);
    LiveSim *t=live_create("configs/x8_design_study.conf",42);
    assert(s && t);
    /* Environmental fault RNG must not alter the arm trajectory. */
    for(int i=0;i<9000;i++) {
        if(i%1000==0) trigger(s,i/1000,i*0.001);
        random_arm(s,i*0.001); random_arm(t,i*0.001);
        assert(s->arm_angle==t->arm_angle);
        assert(s->arm_torque==t->arm_torque);
        assert(fabs(s->arm_speed)<=1.5);
    }
    live_destroy(s); live_destroy(t);
    s=live_create("configs/x8_design_study.conf",43);
    s->control.manual_armed=true;
    random_faults(s,0,1);
    double previous=0;
    unsigned seen=0;
    for(int i=0;i<9;i++) {
        double next=s->next_fault;
        assert(next-previous>=5 && next-previous<=15);
        random_faults(s,next-0.001,1); assert(s->events==(unsigned)i);
        random_faults(s,next,1); assert(s->events==(unsigned)i+1);
        assert(!(seen&(1u<<s->hazard))); seen|=1u<<s->hazard;
        previous=next;
    }
    assert(seen==511);
    random_faults(s,s->next_fault,1);
    assert(s->hazard==9 && s->cycles==1);
    assert(!s->world.imu_valid && !s->world.nav_valid && !s->world.rc_link_up);
    double efficiency[8]; memcpy(efficiency,s->world.phys.eff,sizeof(efficiency));
    random_faults(s,s->next_fault+100,0);
    for(int i=0;i<8;i++) assert(s->world.phys.eff[i]==efficiency[i]);
    live_destroy(s);

    s=live_create("configs/x8_design_study.conf",44);
    uint16_t rc[5]={1500,1500,1000,1500,1000};
    live_step(s,rc,1,0,100);
    Vec3 held=s->input.nav_pos; uint32_t stamp=s->input.nav_ms;
    trigger(s,7,s->tick*0.001);
    s->state.pos.x=50; /* plant truth changes during outage */
    live_step(s,rc,1,0,250);
    assert(!s->input.nav_valid);
    assert(s->input.nav_ms==stamp && s->input.nav_pos.x==held.x);
    assert(s->control.nav_degraded);
    assert(!s->control.manual_armed);
    live_destroy(s);
    puts("Live bridge: random schedule, cumulative damage, independent arm RNG, GPS isolation PASS");
    return 0;
}
