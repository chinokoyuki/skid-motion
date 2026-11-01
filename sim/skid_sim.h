#ifndef SKID_SIM_H
#define SKID_SIM_H

#include "skid_motion.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    float track_mm;
    float free_speed_mms;
    float drive_tau_s;
    float brake_tau_s;
    float coast_tau_s;
    float left_gain;
    float right_gain;
    float static_cmd;
    float gyro_bias_dps;
    float common_mode_disturb_mms;
} skm_sim_config_t;

typedef struct
{
    skm_sim_config_t config;

    float left_speed_mms;
    float right_speed_mms;
    float left_distance_mm;
    float right_distance_mm;
    float yaw_deg;
    float sensor_yaw_deg;
    float yaw_rate_dps;
    float sensor_yaw_rate_dps;

    skm_output_t output;
    float left_delta_mm;
    float right_delta_mm;

    bool stuck;
    bool invert_motor;
    bool invert_gyro;
    bool force_yaw_rate_enable;
    float forced_yaw_rate_dps;
    bool samples_valid;
} skm_sim_t;

void skm_sim_config_default(skm_sim_config_t *config);
void skm_sim_init(skm_sim_t *sim, const skm_sim_config_t *config);
void skm_sim_output(void *user, const skm_output_t *output);
void skm_sim_step(skm_sim_t *sim, float dt_s);
void skm_sim_sample(const skm_sim_t *sim, skm_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif /* SKID_SIM_H */
