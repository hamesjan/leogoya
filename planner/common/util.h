#ifndef LEOGOYA_UTIL_H
#define LEOGOYA_UTIL_H
#include <stdint.h>

/* Deterministic PRNG (xorshift32): the same seed always yields the same painting plan. */
typedef struct
{
    uint32_t state;
} rng_t;

static inline uint32_t rng_next(rng_t *rng)
{
    uint32_t x = rng->state ? rng->state : 0x9E3779B9u;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    rng->state = x;
    return x;
}

/* uniform float in [0, 1) */
static inline float rng_unit(rng_t *rng)
{
    return (rng_next(rng) >> 8) * (1.0f / 16777216.0f);
}

/* uniform int in [0, count) */
static inline int rng_range(rng_t *rng, int count)
{
    return (int)(rng_next(rng) % (uint32_t)count);
}

static inline float clampf(float value, float low, float high)
{
    if (value < low)
    {
        return low;
    }
    if (value > high)
    {
        return high;
    }
    return value;
}

#endif
