#include "ukf_only.h"
#include <assert.h>
#include <math.h>
int main(void){
 float a[2]={179,85},out[2]={0,0};
 assert(fdUkfOnlySelect(a,true,true,true,10000,.000312f,out)==FD_ONLY_VALID);
 assert(out[0]==179 && out[1]==85);
 assert(fdUkfOnlySelect(a,true,true,false,10000,.000312f,out)==FD_ONLY_INVALID);
 assert(out[0]==179 && out[1]==85);
 a[0]=NAN;assert(fdUkfOnlySelect(a,true,true,true,0,.000312f,out)==FD_ONLY_INVALID);
 a[0]=35;assert(fdUkfOnlySelect(a,true,true,true,20001,.000312f,out)==FD_ONLY_INVALID);
 assert(fdUkfOnlySelect(a,false,true,true,0,.000312f,out)==FD_ONLY_INACTIVE);
 assert(fdUkfOnlySelect(a,true,true,true,0,NAN,out)==FD_ONLY_INVALID);
}
