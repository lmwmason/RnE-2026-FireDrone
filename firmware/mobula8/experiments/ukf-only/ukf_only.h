/* Host-only experiment: intentionally not linked into flight firmware. */
#pragma once
#include <stdbool.h>
#include <stdint.h>
typedef enum { FD_ONLY_INACTIVE, FD_ONLY_VALID, FD_ONLY_INVALID } fdOnlyResult;
fdOnlyResult fdUkfOnlySelect(const float attitude[2],bool enabled,bool permitted,bool valid,
    uint32_t ageUs,float dt,float selected[2]);
