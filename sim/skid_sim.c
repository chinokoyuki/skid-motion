#include "skid_sim.h"

#include <math.h>
#include <string.h>

#define SKM_SIM_DEG_PER_RAD 57.29577951308232f

static float skm_sim_drive_target(const skm_sim_t *sim, float command, float gain)
{
    if (fabsf(command) <= fabsf(sim->config.static_cmd))
    {
        return 0.0f;
    }
    return command * sim->config.free_speed_mms * gain +
           sim->config.common_mode_disturb_mms;
}

static float skm_sim_integrate_wheel(float speed, float target, float tau_s,
                                     float dt_s, float *delta_mm)
{
    float decay;

    if (tau_s <= 0.0f)
    {
        *delta_mm = target * dt_s;
        return target;
    }

    decay = expf(-dt_s / tau_s);
    *delta_mm = target * dt_s + (speed - target) * tau_s * (1.0f - decay);
    return target + (speed - target) * decay;
}

void skm_sim_config_default(skm_sim_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    memset(config, 0, sizeof(*config));
    config->track_mm = 120.0f;
    config->free_speed_mms = 300.0f;
    config->drive_tau_s = 0.060f;
    config->brake_tau_s = 0.045f;
    config->coast_tau_s = 0.250f;
    config->left_gain = 1.0f;
    config->right_gain = 1.0f;
    config->static_cmd = 0.08f;
    config->gyro_bias_dps = 0.0f;
    config->common_mode_disturb_mms = 0.0f;
}

void skm_sim_init(skm_sim_t *sim, const skm_sim_config_t *config)
{
    skm_sim_config_t defaults;

    if (sim == NULL)
    {
        return;
    }

    if (config == NULL)
    {
        skm_sim_config_default(&defaults);
        config = &defaults;
    }

    memset(sim, 0, sizeof(*sim));
    sim->config = *config;
    sim->output.mode = SKM_OUTPUT_BRAKE;
    sim->samples_valid = true;
}

void skm_sim_output(void *user, const skm_output_t *output)
{
    skm_sim_t *sim = (skm_sim_t *)user;

    if (sim == NULL || output == NULL)
    {
        return;
    }
    sim->output = *output;
}

void skm_sim_step(skm_sim_t *sim, float dt_s)
{
    float left_target = 0.0f;
    float right_target = 0.0f;
    float tau_s;
    float measured_gz;

    if (sim == NULL)
    {
        return;
    }

    sim->left_delta_mm = 0.0f;
    sim->right_delta_mm = 0.0f;
    if (!isfinite(dt_s) || dt_s <= 0.0f)
    {
        return;
    }

    if (sim->stuck)
    {
        sim->left_speed_mms = 0.0f;
        sim->right_speed_mms = 0.0f;
    }
    else
    {
        if (sim->output.mode == SKM_OUTPUT_DRIVE)
        {
            float left_command = sim->output.left;
            float right_command = sim->output.right;

            if (sim->invert_motor)
            {
                left_command = -left_command;
                right_command = -right_command;
            }
            left_target = skm_sim_drive_target(sim, left_command,
                                               sim->config.left_gain);
            right_target = skm_sim_drive_target(sim, right_command,
                                                sim->config.right_gain);
            tau_s = sim->config.drive_tau_s;
        }
        else
        {
            tau_s = (sim->output.mode == SKM_OUTPUT_COAST)
                        ? sim->config.coast_tau_s
                        : sim->config.brake_tau_s;
        }

        sim->left_speed_mms = skm_sim_integrate_wheel(
            sim->left_speed_mms, left_target, tau_s, dt_s, &sim->left_delta_mm);
        sim->right_speed_mms = skm_sim_integrate_wheel(
            sim->right_speed_mms, right_target, tau_s, dt_s, &sim->right_delta_mm);
    }

    sim->left_distance_mm += sim->left_delta_mm;
    sim->right_distance_mm += sim->right_delta_mm;

    if (fabsf(sim->config.track_mm) > 1.0e-6f)
    {
        sim->yaw_deg += (sim->right_delta_mm - sim->left_delta_mm) *
                        SKM_SIM_DEG_PER_RAD / sim->config.track_mm;
        sim->yaw_rate_dps = (sim->right_speed_mms - sim->left_speed_mms) *
                            SKM_SIM_DEG_PER_RAD / sim->config.track_mm;
    }
    else
    {
        sim->yaw_rate_dps = 0.0f;
    }

    measured_gz = sim->force_yaw_rate_enable ? sim->forced_yaw_rate_dps
                                             : sim->yaw_rate_dps;
    if (sim->invert_gyro)
    {
        measured_gz = -measured_gz;
    }
    sim->sensor_yaw_rate_dps = measured_gz + sim->config.gyro_bias_dps;
    sim->sensor_yaw_deg += sim->sensor_yaw_rate_dps * dt_s;
}

void skm_sim_sample(const skm_sim_t *sim, skm_sample_t *sample)
{
    if (sim == NULL || sample == NULL)
    {
        return;
    }

    sample->left_delta_mm = sim->left_delta_mm;
    sample->right_delta_mm = sim->right_delta_mm;
    sample->left_speed_mms = sim->left_speed_mms;
    sample->right_speed_mms = sim->right_speed_mms;
    sample->yaw_deg = sim->sensor_yaw_deg;
    sample->yaw_rate_dps = sim->sensor_yaw_rate_dps;
    sample->valid = sim->samples_valid;
}
