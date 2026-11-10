#ifndef LEOGOYA_UTIL_H
#define LEOGOYA_UTIL_H
#include <stdint.h>

/* Deterministic PRNG so a given reference always yields the same painting plan
 * (lets you resume after a power cut by re-planning and skipping N strokes). */
typedef struct { uint32_t s; } rng_t;

static inline uint32_t rng_next(rng_t *r) {
    uint32_t x = r->s ? r->s : 0x9E3779B9u;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    return r->s = x;
}
static inline float rng_unit(rng_t *r) { return (rng_next(r) >> 8) * (1.0f / 16777216.0f); }
static inline int rng_range(rng_t *r, int n) { return (int)(rng_next(r) % (uint32_t)n); }

static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : v > hi ? hi : v; }

#endif
