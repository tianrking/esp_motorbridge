#pragma once

#include <stdint.h>

static inline uint32_t damiao_float_to_uint(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float clipped = x;
    if (clipped < x_min) {
        clipped = x_min;
    }
    if (clipped > x_max) {
        clipped = x_max;
    }
    return (uint32_t)(((clipped - x_min) * ((float)((1u << bits) - 1u))) / span);
}

static inline float damiao_uint_to_float(uint32_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return ((float)x) * span / ((float)((1u << bits) - 1u)) + x_min;
}
