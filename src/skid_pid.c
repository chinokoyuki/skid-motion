#include "skid_pid.h"
#include "skid_math.h"

void skm_pid_init(skm_internal_pid_t *state)
{
    skm_pid_reset(state);
}

void skm_pid_reset(skm_internal_pid_t *state)
{
    state->integral = 0.0f;
    state->output = 0.0f;
}

float skm_pid_calculate(skm_internal_pid_t *state,
                        const skm_pid_config_t *config,
                        float target,
                        float measurement,
                        float measurement_rate,
                        float dt_s)
{
    float error = target - measurement;
    float old_integral = state->integral;
    float candidate = old_integral;
    float derivative = -config->kd * measurement_rate;
    float unsaturated;

    if (config->i_limit <= 0.0f || config->ki == 0.0f)
    {
        candidate = 0.0f;
    }
    else if (dt_s > 0.0f)
    {
        candidate = skm_clampf(old_integral + error * dt_s, config->i_limit);
    }

    unsaturated = config->kp * error + config->ki * candidate + derivative;
    if (!((unsaturated > config->out_limit && error > 0.0f) ||
          (unsaturated < -config->out_limit && error < 0.0f)))
    {
        state->integral = candidate;
    }

    state->output = skm_clampf(config->kp * error +
                                   config->ki * state->integral + derivative,
                               config->out_limit);
    return state->output;
}
