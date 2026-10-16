#ifndef FC_ALLOCATOR_H
#define FC_ALLOCATOR_H
#include <stdbool.h>
#include <stdint.h>
#define FC_MOTORS 8
/* Body FLU, vertical rotors. SI units. Controller estimates, not plant truth. */
typedef struct { float x, y, thrust, yaw_torque; } FcMotor;
typedef struct {
    bool initialized;
    float B[4][8], weight[4], u[8];
} FcAllocator;
typedef struct {
    float u[8], achieved[4], residual[4];
    unsigned iterations;
    bool converged; /* Solver convergence does not imply feasible flight. */
} FcAllocation;
bool fc_allocator_init(FcAllocator *a, const FcMotor motors[8], const float weights[4]);
bool fc_allocate(FcAllocator *a, const float request[4], uint8_t failed,
                 FcAllocation *out);
#endif
