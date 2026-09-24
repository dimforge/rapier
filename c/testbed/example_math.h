#ifndef RAPIER_EXAMPLE_MATH_H
#define RAPIER_EXAMPLE_MATH_H
#include "rapier_math.h"
#include <stdlib.h>

/* A fixed PCG stream makes the C examples reproducible across libc versions. */
static inline RAPIER_TYPE(Real) exampleRandom(uint64_t *state) {
    uint64_t old = *state;
    *state = old * UINT64_C(6364136223846793005) + UINT64_C(1442695040888963407);
    uint32_t value = (uint32_t)(((old >> 18) ^ old) >> 27);
    uint32_t rotation = (uint32_t)(old >> 59);
    return (RAPIER_TYPE(Real))(((value >> rotation) | (value << ((-rotation) & 31))) >> 8) /
           (RAPIER_TYPE(Real))16777216;
}
#endif
