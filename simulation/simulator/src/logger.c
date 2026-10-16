#include "logger.h"

int logger_open(Logger *lg, const char *path) {
    lg->fp = fopen(path, "w");
    return lg->fp != NULL;
}

void logger_write_header(Logger *lg) {
    fprintf(lg->fp,
        "t,px,py,pz,vx,vy,vz,qw,qx,qy,qz,wx,wy,wz,"
        "rx,ry,rz,wind_x,wind_y,wind_z,tq_x,tq_y,"
        "pwm1,pwm2,pwm3,pwm4,T1,T2,T3,T4,eff1,eff2,eff3,eff4,"
        "phase,wp,link,imu_fault,arm,fault,"
        "ukf_qw,ukf_qx,ukf_qy,ukf_qz,fhat_x,fhat_y,fhat_z,"
        "th1_x,th1_y,th1_z,th2_x,th2_y,th2_z,overruns,"
        "rc_roll,rc_pitch,rc_thr,rc_yaw,rc_aux,lap,events_fired,crashed,impact_vz,pwm5,pwm6,pwm7,pwm8,T5,T6,T7,T8,eff5,eff6,eff7,eff8,motor_count,motor_mask,nav_valid,imu_valid,allocator_converged,allocation_residual\n");
}

void logger_write(Logger *lg, const LogRow *r) {
    const DroneState *s = &r->s;
    const FwOut *f = &r->fw;
    fprintf(lg->fp,
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,"
        "%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,"
        "%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,"
        "%d,%d,%d,%d,%d,%d,"
        "%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,"
        "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%u,"
        "%u,%u,%u,%u,%u,%d,%d,%d,%.4f,%u,%u,%u,%u,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d,%d,%.5f\n",
        r->t, s->pos.x, s->pos.y, s->pos.z, s->vel.x, s->vel.y, s->vel.z,
        s->q.w, s->q.x, s->q.y, s->q.z, s->w.x, s->w.y, s->w.z,
        r->ref_pos.x, r->ref_pos.y, r->ref_pos.z,
        r->wind.x, r->wind.y, r->wind.z, r->ext_torque.x, r->ext_torque.y,
        f->pwm_us[0], f->pwm_us[1], f->pwm_us[2], f->pwm_us[3],
        r->thrust[0], r->thrust[1], r->thrust[2], r->thrust[3],
        r->eff[0], r->eff[1], r->eff[2], r->eff[3],
        r->phase, r->wp, r->link_up, r->imu_fault, f->arm, f->fault_state,
        f->ukf_q[0], f->ukf_q[1], f->ukf_q[2], f->ukf_q[3],
        f->adrc_fhat[0], f->adrc_fhat[1], f->adrc_fhat[2],
        f->mrac_theta1[0], f->mrac_theta1[1], f->mrac_theta1[2],
        f->mrac_theta2[0], f->mrac_theta2[1], f->mrac_theta2[2], f->sched_overruns,
        r->rc[0], r->rc[1], r->rc[2], r->rc[3], r->rc[4],
        r->lap, r->events_fired, r->crashed, r->impact_vz,
        f->pwm_us[4], f->pwm_us[5], f->pwm_us[6], f->pwm_us[7],
        r->thrust[4], r->thrust[5], r->thrust[6], r->thrust[7],
        r->eff[4], r->eff[5], r->eff[6], r->eff[7],
        r->motor_count, r->motor_mask, r->nav_valid, r->imu_valid,
        r->allocator_converged, r->allocation_residual);
}

void logger_close(Logger *lg) {
    if (lg->fp) fclose(lg->fp);
}
