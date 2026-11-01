#include <stdio.h>
#include <stdlib.h>

#include "skid_motion.h"
#include "skid_sim.h"

#define SIM_DT_S 0.001f
#define STRAIGHT_MAX_TICKS 15000U
#define TURN_MAX_TICKS 20000U

static const char *result_name(skm_result_t result)
{
    switch (result)
    {
    case SKM_RESULT_NONE:
        return "NONE";
    case SKM_RESULT_RUNNING:
        return "RUNNING";
    case SKM_RESULT_OK:
        return "OK";
    case SKM_RESULT_ABORTED:
        return "ABORTED";
    case SKM_RESULT_TIMEOUT:
        return "TIMEOUT";
    case SKM_RESULT_STALLED:
        return "STALLED";
    case SKM_RESULT_OVERSHOOT:
        return "OVERSHOOT";
    case SKM_RESULT_DIVERGED:
        return "DIVERGED";
    case SKM_RESULT_SENSOR_FAULT:
        return "SENSOR_FAULT";
    default:
        return "UNKNOWN";
    }
}

static bool run_action(skm_t *motion, skm_sim_t *sim, const char *name,
                       float target, bool turn, unsigned int max_ticks)
{
    skm_api_result_t api_result;
    skm_status_t status;
    skm_sample_t sample;
    unsigned int ticks = 0U;

    if (turn)
    {
        api_result = skm_start_turn(motion, target);
    }
    else
    {
        api_result = skm_start_straight(motion, target);
    }
    if (api_result != SKM_API_OK)
    {
        printf("%s: start failed (%d)\n", name, (int)api_result);
        return false;
    }

    while (skm_is_busy(motion) && ticks < max_ticks)
    {
        skm_sim_sample(sim, &sample);
        skm_tick(motion, &sample, SIM_DT_S);
        skm_sim_step(sim, SIM_DT_S);
        ticks++;
    }

    if (skm_is_busy(motion))
    {
        skm_abort(motion, SKM_STOP_BRAKE);
        skm_sim_sample(sim, &sample);
        skm_tick(motion, &sample, SIM_DT_S);
        skm_sim_step(sim, SIM_DT_S);
    }

    skm_get_status(motion, &status);
    if (turn)
    {
        printf("%s: result=%s, remaining=%.3f deg, ticks=%u\n",
               name, result_name(status.result), (double)status.remaining_deg, ticks);
    }
    else
    {
        printf("%s: result=%s, remaining=%.3f mm, ticks=%u\n",
               name, result_name(status.result), (double)status.remaining_mm, ticks);
    }

    return !status.busy && status.result == SKM_RESULT_OK;
}

int main(void)
{
    skm_config_t motion_config;
    skm_sim_config_t sim_config;
    skm_learning_t learning;
    skm_sim_t sim;
    skm_t motion;
    bool all_ok;

    skm_config_default(&motion_config);
    skm_sim_config_default(&sim_config);
    skm_sim_init(&sim, &sim_config);
    if (skm_init(&motion, &motion_config, skm_sim_output, &sim) != SKM_API_OK)
    {
        fprintf(stderr, "Failed to initialize skid motion controller\n");
        return EXIT_FAILURE;
    }

    all_ok = run_action(&motion, &sim, "straight 250 mm", 250.0f, false,
                        STRAIGHT_MAX_TICKS);
    if (!run_action(&motion, &sim, "left turn 20 deg", 20.0f, true,
                    TURN_MAX_TICKS))
    {
        all_ok = false;
    }
    if (!run_action(&motion, &sim, "small left turn 3 deg", 3.0f, true,
                    TURN_MAX_TICKS))
    {
        all_ok = false;
    }

    skm_get_learning(&motion, &learning);
    printf("learning: straight_brake=%.4f s, turn_brake=%.4f s, "
           "heading_trim=%.3f mm/s, hop_gain=%.3f dps\n",
           (double)learning.straight_brake_s, (double)learning.turn_brake_s,
           (double)learning.heading_trim_mms, (double)learning.hop_gain_dps);

    return all_ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
