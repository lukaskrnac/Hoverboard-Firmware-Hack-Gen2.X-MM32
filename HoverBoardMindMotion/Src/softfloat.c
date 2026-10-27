//Soft float size: libgcc for the Cortex-M0 has separate 1.1 KB implementations of float add and
//subtract. In IEEE arithmetic a - b is exactly a + (-b) (the sign bit flipped), so subtraction calls the
//addition: ~1.1 KB less flash (of 30 KB). These definitions take the place of libgcc's.
#include <stdint.h>
#include <string.h>

float __aeabi_fadd(float a, float b);
#define KEEP __attribute__((used, externally_visible))    //the calls appear after LTO: keep them
KEEP float __aeabi_fsub(float a, float b);
KEEP float __aeabi_frsub(float a, float b);
KEEP float __subsf3(float a, float b);

static float negF(float b){ uint32_t u; memcpy(&u, &b, 4); u ^= 0x80000000u; memcpy(&b, &u, 4); return b; }

float __aeabi_fsub(float a, float b){ return __aeabi_fadd(a, negF(b)); }
float __aeabi_frsub(float a, float b){ return __aeabi_fadd(b, negF(a)); }
float __subsf3(float a, float b){ return __aeabi_fadd(a, negF(b)); }
