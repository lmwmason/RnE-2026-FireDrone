#ifndef LOGGER_H
#define LOGGER_H

#include <stdio.h>
#include "physics.h"
#include "fw_host.h"

typedef struct {
    FILE *fp;
} Logger;

typedef struct {
    double      t;
    DroneState  s;
    Vec3        ref_pos;
    Vec3        wind;
    Vec3        ext_torque;
    double      eff[8];
    double      thrust[8];
    int         phase, wp, link_up;
    int         imu_fault;
    int         lap, events_fired, crashed;
    int         motor_count, motor_mask, nav_valid, imu_valid, allocator_converged;
    double      allocation_residual;
    double      impact_vz;
    FwOut       fw;
    uint16_t    rc[5];
} LogRow;

int  logger_open(Logger *lg, const char *path);
void logger_write_header(Logger *lg);
void logger_write(Logger *lg, const LogRow *r);
void logger_close(Logger *lg);

#endif
