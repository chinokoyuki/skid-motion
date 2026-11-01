#ifndef SKID_MATH_H
#define SKID_MATH_H

static inline float skm_clampf(float value, float limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

static inline float skm_move_toward(float value, float target, float max_delta)
{
    if (value < target - max_delta)
    {
        return value + max_delta;
    }
    if (value > target + max_delta)
    {
        return value - max_delta;
    }
    return target;
}

static inline float skm_signf(float value)
{
    return (value < 0.0f) ? -1.0f : 1.0f;
}

static inline float skm_maxf(float a, float b)
{
    return (a > b) ? a : b;
}

#endif /* SKID_MATH_H */
