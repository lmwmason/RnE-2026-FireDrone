#include "ukf_only.h"
#include <string.h>
static bool finiteFloat(float v){uint32_t bits;memcpy(&bits,&v,sizeof bits);return (bits&0x7f800000U)!=0x7f800000U;}
fdOnlyResult fdUkfOnlySelect(const float attitude[2],bool enabled,bool permitted,bool valid,
    uint32_t ageUs,float dt,float selected[2])
{
    if(!enabled || !permitted)return FD_ONLY_INACTIVE;
    if(!valid || ageUs>20000 || !finiteFloat(dt) || dt<=0 || dt>.002f ||
        !finiteFloat(attitude[0]) || !finiteFloat(attitude[1]))return FD_ONLY_INVALID;
    /* No baseline input, disagreement/envelope switching, or baseline blending. */
    selected[0]=attitude[0];selected[1]=attitude[1];return FD_ONLY_VALID;
}
