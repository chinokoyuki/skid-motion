#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "skid_motion.h"
#include "skid_sim.h"

#define CHECK(expression)                                                     \
    do                                                                        \
    {                                                                         \
        if (!(expression))                                                    \
        {                                                                     \
            fprintf(stderr, "%s:%d: check failed: %s\n",                    \
                    __FILE__, __LINE__, #expression);                         \
            return false;                                                     \
        }                                                                     \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                               \
    CHECK(fabsf((actual) - (expected)) <= (tolerance))

typedef struct
{
    skm_t motion;
    skm_sim_t sim;
} rig_t;

typedef struct
{
    skm_result_t result;
    unsigned int ticks;
    float average_distance_mm;
    float yaw_deg;
    float cruise_yaw_rms_deg;
    float start_yaw_deg;
    float yaw_delta_deg;
    uint16_t hop_count;
    bool saw_approach;
} action_summary_t;

typedef struct
{
    unsigned int count;
    skm_output_t last;
} collector_t;

static void collect_output(void *user, const skm_output_t *output)
{
    collector_t *collector = (collector_t *)user;
    collector->count++;
    collector->last = *output;
}

static skm_sample_t zero_sample(void)
{
    skm_sample_t sample;
    memset(&sample, 0, sizeof(sample));
    sample.valid = true;
    return sample;
}

static bool rig_init(rig_t *rig,
                     const skm_config_t *motion_config,
                     const skm_sim_config_t *sim_config)
{
    skm_sim_init(&rig->sim, sim_config);
    return skm_init(&rig->motion, motion_config, skm_sim_output, &rig->sim) ==
           SKM_API_OK;
}

static bool run_current_action(rig_t *rig, bool turn, unsigned int max_ticks,
                               float dt_s, action_summary_t *summary)
{
    skm_sample_t sample;
    skm_status_t status;
    unsigned int ticks = 0U;
    double yaw_square_sum = 0.0;
    unsigned int yaw_samples = 0U;
    bool captured_start_yaw = false;
    float start_true_yaw = 0.0f;

    memset(summary, 0, sizeof(*summary));
    while (skm_is_busy(&rig->motion) && ticks < max_ticks)
    {
        skm_get_status(&rig->motion, &status);
        if (!captured_start_yaw)
        {
            summary->start_yaw_deg = rig->sim.sensor_yaw_deg;
            start_true_yaw = rig->sim.yaw_deg;
            captured_start_yaw = true;
        }
        if (status.phase == SKM_PHASE_TURN_APPROACH)
        {
            summary->saw_approach = true;
        }

        skm_sim_sample(&rig->sim, &sample);
        skm_tick(&rig->motion, &sample, dt_s);
        skm_sim_step(&rig->sim, dt_s);
        ticks++;

        if (!turn && fabsf(rig->sim.left_speed_mms) > 70.0f &&
            fabsf(rig->sim.right_speed_mms) > 70.0f)
        {
            float relative_yaw = sample.yaw_deg - summary->start_yaw_deg;
            yaw_square_sum += (double)relative_yaw * (double)relative_yaw;
            yaw_samples++;
        }
    }

    skm_get_status(&rig->motion, &status);
    summary->result = status.result;
    summary->ticks = ticks;
    summary->average_distance_mm =
        0.5f * (rig->sim.left_distance_mm + rig->sim.right_distance_mm);
    summary->yaw_deg = rig->sim.yaw_deg;
    summary->yaw_delta_deg = rig->sim.yaw_deg - start_true_yaw;
    summary->hop_count = status.hop_count;
    if (yaw_samples > 0U)
    {
        summary->cruise_yaw_rms_deg =
            (float)sqrt(yaw_square_sum / (double)yaw_samples);
    }
    return true;
}

static bool run_action(rig_t *rig, bool turn, float target,
                       unsigned int max_ticks, float dt_s,
                       action_summary_t *summary)
{
    skm_api_result_t result = turn ? skm_start_turn(&rig->motion, target)
                                   : skm_start_straight(&rig->motion, target);
    if (result != SKM_API_OK)
    {
        return false;
    }
    return run_current_action(rig, turn, max_ticks, dt_s, summary);
}

static bool test_configuration_and_idle_output(void)
{
    skm_config_t config;
    collector_t collector;
    skm_sample_t sample = zero_sample();
    skm_t motion;

    skm_config_default(&config);
    memset(&collector, 0, sizeof(collector));
    CHECK(skm_init(&motion, &config, collect_output, &collector) == SKM_API_OK);
    CHECK(!skm_is_busy(&motion));

    skm_tick(&motion, &sample, 0.001f);
    CHECK(collector.count == 1U);
    CHECK(collector.last.mode == SKM_OUTPUT_BRAKE);

    config.cruise_mms = NAN;
    CHECK(skm_init(&motion, &config, collect_output, &collector) ==
          SKM_API_INVALID);
    return true;
}

static bool test_api_busy_abort_and_sensor_fault(void)
{
    skm_config_t config;
    collector_t collector;
    skm_sample_t sample = zero_sample();
    skm_t motion;
    skm_learning_t before;
    skm_learning_t after;
    skm_status_t status;

    skm_config_default(&config);
    memset(&collector, 0, sizeof(collector));
    CHECK(skm_init(&motion, &config, collect_output, &collector) == SKM_API_OK);

    CHECK(skm_start_straight(&motion, 100.0f) == SKM_API_OK);
    CHECK(collector.count == 0U);
    CHECK(skm_start_turn(&motion, 10.0f) == SKM_API_BUSY);
    CHECK(skm_start_straight(&motion, NAN) == SKM_API_BUSY);
    skm_tick(&motion, &sample, 0.001f);
    CHECK(collector.count == 1U);
    CHECK(collector.last.mode == SKM_OUTPUT_DRIVE);

    skm_get_learning(&motion, &before);
    skm_abort(&motion, SKM_STOP_COAST);
    skm_tick(&motion, &sample, 0.001f);
    skm_get_status(&motion, &status);
    CHECK(status.result == SKM_RESULT_ABORTED);
    CHECK(collector.last.mode == SKM_OUTPUT_COAST);
    skm_get_learning(&motion, &after);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);

    CHECK(skm_start_wheel_speed(&motion, 0.0f, 0.0f) == SKM_API_OK);
    CHECK(skm_is_busy(&motion));
    skm_tick(&motion, &sample, 0.001f);
    CHECK(collector.last.mode == SKM_OUTPUT_DRIVE);
    skm_abort(&motion, SKM_STOP_BRAKE);
    skm_tick(&motion, &sample, 0.001f);

    CHECK(skm_start_straight(&motion, 50.0f) == SKM_API_OK);
    skm_tick(&motion, &sample, 0.001f);
    skm_get_learning(&motion, &before);
    sample.valid = false;
    skm_tick(&motion, &sample, 0.001f);
    skm_get_status(&motion, &status);
    CHECK(status.result == SKM_RESULT_SENSOR_FAULT);
    CHECK(collector.last.mode == SKM_OUTPUT_BRAKE);
    skm_get_learning(&motion, &after);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);

    CHECK(skm_start_straight(&motion, 0.0f) == SKM_API_INVALID);
    CHECK(skm_reset_learning(&motion) == SKM_API_OK);
    return true;
}

static bool test_wheel_speed_mode(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    skm_status_t status;
    unsigned int i;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));
    CHECK(skm_start_wheel_speed(&rig.motion, 80.0f, 120.0f) == SKM_API_OK);

    for (i = 0U; i < 2000U; ++i)
    {
        skm_sample_t sample;
        skm_sim_sample(&rig.sim, &sample);
        skm_tick(&rig.motion, &sample, 0.001f);
        skm_sim_step(&rig.sim, 0.001f);
        CHECK(fabsf(rig.sim.output.left) <= 1.0f);
        CHECK(fabsf(rig.sim.output.right) <= 1.0f);
    }

    skm_get_status(&rig.motion, &status);
    CHECK(status.busy);
    CHECK_NEAR(rig.sim.left_speed_mms, 80.0f, 4.0f);
    CHECK_NEAR(rig.sim.right_speed_mms, 120.0f, 4.0f);
    CHECK(rig.sim.yaw_deg > 0.0f);
    skm_abort(&rig.motion, SKM_STOP_BRAKE);
    return true;
}

static bool test_straight_distance_and_direction(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));

    CHECK(run_action(&rig, false, 250.0f, 6000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_OK);
    CHECK_NEAR(summary.average_distance_mm, 250.0f, 2.1f);
    CHECK_NEAR(summary.yaw_deg, 0.0f, 0.8f);

    CHECK(run_action(&rig, false, -150.0f, 5000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_OK);
    CHECK_NEAR(summary.average_distance_mm, 100.0f, 2.2f);
    return true;
}

static bool test_overshoot_is_reported_without_reverse(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    sim_config.brake_tau_s = 0.080f;
    config.wheel_speed_max_mms = 400.0f;
    config.cruise_mms = 250.0f;
    config.position_kp = 20.0f;
    config.straight_brake_init_s = 0.005f;
    config.straight_brake_min_s = 0.001f;
    CHECK(rig_init(&rig, &config, &sim_config));

    CHECK(run_action(&rig, false, 120.0f, 5000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_OVERSHOOT);
    CHECK(summary.average_distance_mm > 122.0f);
    CHECK(summary.average_distance_mm < 160.0f);
    return true;
}

static bool test_learning_can_be_saved_and_restored(void)
{
    skm_config_t config;
    collector_t collector;
    skm_t motion;
    skm_learning_t saved;
    skm_learning_t loaded;

    skm_config_default(&config);
    memset(&collector, 0, sizeof(collector));
    CHECK(skm_init(&motion, &config, collect_output, &collector) == SKM_API_OK);

    saved.straight_brake_s = 0.045f;
    saved.turn_brake_s = 0.070f;
    saved.heading_trim_mms = -3.5f;
    saved.hop_gain_dps = 130.0f;
    CHECK(skm_set_learning(&motion, &saved) == SKM_API_OK);
    skm_get_learning(&motion, &loaded);
    CHECK(memcmp(&saved, &loaded, sizeof(saved)) == 0);

    saved.hop_gain_dps = NAN;
    CHECK(skm_set_learning(&motion, &saved) == SKM_API_INVALID);
    skm_get_learning(&motion, &loaded);
    CHECK_NEAR(loaded.hop_gain_dps, 130.0f, 0.001f);

    CHECK(skm_start_turn(&motion, 10.0f) == SKM_API_OK);
    CHECK(skm_set_learning(&motion, &saved) == SKM_API_BUSY);
    return true;
}

static bool test_brake_learning_convergence(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;
    skm_learning_t learning;
    unsigned int i;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));

    for (i = 0U; i < 4U; ++i)
    {
        CHECK(run_action(&rig, false, 160.0f, 5000U, 0.001f, &summary));
        CHECK(summary.result == SKM_RESULT_OK);
    }
    skm_get_learning(&rig.motion, &learning);
    CHECK(learning.straight_brake_s >= config.straight_brake_min_s);
    CHECK(learning.straight_brake_s <= config.straight_brake_max_s);
    CHECK_NEAR(learning.straight_brake_s, sim_config.brake_tau_s, 0.018f);
    return true;
}

static bool test_abort_does_not_learn(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    skm_sample_t sample;
    skm_learning_t before;
    skm_learning_t after;
    unsigned int i;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));
    CHECK(skm_start_straight(&rig.motion, 300.0f) == SKM_API_OK);
    for (i = 0U; i < 100U; ++i)
    {
        skm_sim_sample(&rig.sim, &sample);
        skm_tick(&rig.motion, &sample, 0.001f);
        skm_sim_step(&rig.sim, 0.001f);
    }
    skm_get_learning(&rig.motion, &before);
    skm_abort(&rig.motion, SKM_STOP_BRAKE);
    skm_sim_sample(&rig.sim, &sample);
    skm_tick(&rig.motion, &sample, 0.001f);
    skm_get_learning(&rig.motion, &after);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);
    return true;
}

static bool test_turn_accuracy_and_hop_spacing(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));

    CHECK(run_action(&rig, true, 20.0f, 6000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_OK);
    CHECK_NEAR(summary.yaw_deg, 20.0f, 1.01f);
    CHECK(summary.saw_approach);

    CHECK(run_action(&rig, true, 3.0f, 4000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_OK);
    CHECK_NEAR(summary.yaw_deg, 23.0f, 1.01f);
    CHECK(!summary.saw_approach);
    CHECK(summary.hop_count > 0U);
    return true;
}

static bool test_stuck_turn_reports_stalled(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;
    skm_learning_t before;
    skm_learning_t after;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));
    rig.sim.stuck = true;
    skm_get_learning(&rig.motion, &before);

    CHECK(run_action(&rig, true, 3.0f, 2000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_STALLED);
    skm_get_learning(&rig.motion, &after);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);
    return true;
}

static bool test_reversed_motor_reports_diverged(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;
    skm_learning_t before;
    skm_learning_t after;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));
    rig.sim.invert_motor = true;
    skm_get_learning(&rig.motion, &before);

    CHECK(run_action(&rig, true, 10.0f, 5000U, 0.001f, &summary));
    CHECK(summary.result == SKM_RESULT_DIVERGED);
    skm_get_learning(&rig.motion, &after);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);
    return true;
}

static bool test_no_still_gate_blocks_completion(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    skm_sample_t sample;
    skm_status_t status;
    skm_learning_t before;
    skm_learning_t after;
    unsigned int i;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig, &config, &sim_config));
    rig.sim.force_yaw_rate_enable = true;
    rig.sim.forced_yaw_rate_dps = 20.0f;
    skm_get_learning(&rig.motion, &before);
    CHECK(skm_start_turn(&rig.motion, 3.0f) == SKM_API_OK);

    for (i = 0U; i < 600U && skm_is_busy(&rig.motion); ++i)
    {
        skm_sim_sample(&rig.sim, &sample);
        skm_tick(&rig.motion, &sample, 0.001f);
        skm_sim_step(&rig.sim, 0.001f);
    }
    skm_get_status(&rig.motion, &status);
    CHECK(status.busy);
    skm_abort(&rig.motion, SKM_STOP_BRAKE);
    skm_sim_sample(&rig.sim, &sample);
    skm_tick(&rig.motion, &sample, 0.001f);
    skm_get_learning(&rig.motion, &after);
    CHECK(memcmp(&before, &after, sizeof(before)) == 0);
    return true;
}

static bool test_heading_trim_reduces_systematic_bias(void)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t first;
    action_summary_t second;
    skm_learning_t learning;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    sim_config.left_gain = 0.82f;
    sim_config.right_gain = 1.0f;
    CHECK(rig_init(&rig, &config, &sim_config));

    CHECK(run_action(&rig, false, 350.0f, 7000U, 0.001f, &first));
    CHECK(first.result == SKM_RESULT_OK);
    CHECK(run_action(&rig, false, 350.0f, 7000U, 0.001f, &second));
    CHECK(second.result == SKM_RESULT_OK);

    skm_get_learning(&rig.motion, &learning);
    CHECK(learning.heading_trim_mms < -0.1f);
    CHECK(second.cruise_yaw_rms_deg < first.cruise_yaw_rms_deg);
    return true;
}

static bool test_translation_correction_reduces_drift(void)
{
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t corrected;
    action_summary_t uncorrected;
    rig_t rig_a;
    rig_t rig_b;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    sim_config.common_mode_disturb_mms = 24.0f;

    CHECK(rig_init(&rig_a, &config, &sim_config));
    CHECK(run_action(&rig_a, true, 15.0f, 6000U, 0.001f, &corrected));
    CHECK(corrected.result == SKM_RESULT_OK);

    config.translation_kp = 0.0f;
    CHECK(rig_init(&rig_b, &config, &sim_config));
    CHECK(run_action(&rig_b, true, 15.0f, 6000U, 0.001f, &uncorrected));
    CHECK(uncorrected.result == SKM_RESULT_OK);

    CHECK(fabsf(corrected.average_distance_mm) <
          fabsf(uncorrected.average_distance_mm));
    CHECK_NEAR(corrected.yaw_deg, 15.0f, 1.01f);
    return true;
}

static bool run_deterministic_sequence(float *distance, float *learning_value,
                                       unsigned int *ticks)
{
    rig_t rig;
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t summary;
    skm_learning_t learning;
    unsigned int total_ticks = 0U;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    sim_config.left_gain = 0.9f;
    if (!rig_init(&rig, &config, &sim_config))
    {
        return false;
    }
    if (!run_action(&rig, false, 180.0f, 5000U, 0.001f, &summary))
    {
        return false;
    }
    if (summary.result != SKM_RESULT_OK)
    {
        return false;
    }
    total_ticks += summary.ticks;
    if (!run_action(&rig, true, 12.0f, 5000U, 0.001f, &summary))
    {
        return false;
    }
    if (summary.result != SKM_RESULT_OK)
    {
        return false;
    }
    total_ticks += summary.ticks;
    skm_get_learning(&rig.motion, &learning);
    *distance = summary.average_distance_mm;
    *learning_value = learning.hop_gain_dps;
    *ticks = total_ticks;
    return true;
}

static bool test_deterministic_replay(void)
{
    float distance_a;
    float distance_b;
    float learning_a;
    float learning_b;
    unsigned int ticks_a;
    unsigned int ticks_b;

    CHECK(run_deterministic_sequence(&distance_a, &learning_a, &ticks_a));
    CHECK(run_deterministic_sequence(&distance_b, &learning_b, &ticks_b));
    CHECK(distance_a == distance_b);
    CHECK(learning_a == learning_b);
    CHECK(ticks_a == ticks_b);
    return true;
}

static bool test_period_independence(void)
{
    skm_config_t config;
    skm_sim_config_t sim_config;
    action_summary_t fast;
    action_summary_t standard;
    rig_t rig_fast;
    rig_t rig_standard;

    skm_config_default(&config);
    skm_sim_config_default(&sim_config);
    CHECK(rig_init(&rig_fast, &config, &sim_config));
    CHECK(rig_init(&rig_standard, &config, &sim_config));

    CHECK(run_action(&rig_fast, false, 120.0f, 9000U, 0.0005f, &fast));
    CHECK(run_action(&rig_standard, false, 120.0f, 5000U, 0.001f, &standard));
    CHECK(fast.result == SKM_RESULT_OK);
    CHECK(standard.result == SKM_RESULT_OK);
    CHECK_NEAR(fast.average_distance_mm, standard.average_distance_mm, 1.5f);

    CHECK(run_action(&rig_fast, true, 12.0f, 20000U, 0.0005f, &fast));
    CHECK(run_action(&rig_standard, true, 12.0f, 12000U, 0.001f, &standard));
    CHECK(fast.result == SKM_RESULT_OK);
    CHECK(standard.result == SKM_RESULT_OK);
    CHECK_NEAR(fast.yaw_delta_deg, 12.0f, 1.01f);
    CHECK_NEAR(standard.yaw_delta_deg, 12.0f, 1.01f);
    return true;
}

static bool test_timeout_outputs_brake(void)
{
    skm_config_t config;
    collector_t collector;
    skm_sample_t sample = zero_sample();
    skm_status_t status;
    skm_t motion;
    unsigned int i;

    skm_config_default(&config);
    memset(&collector, 0, sizeof(collector));
    CHECK(skm_init(&motion, &config, collect_output, &collector) == SKM_API_OK);
    CHECK(skm_start_straight(&motion, 100.0f) == SKM_API_OK);

    for (i = 0U; i < 102U && skm_is_busy(&motion); ++i)
    {
        skm_tick(&motion, &sample, 0.1f);
    }
    skm_get_status(&motion, &status);
    CHECK(status.result == SKM_RESULT_TIMEOUT);
    CHECK(collector.last.mode == SKM_OUTPUT_BRAKE);
    return true;
}

static bool test_brake_and_coast_are_distinct(void)
{
    skm_sim_config_t config;
    skm_sim_t brake_sim;
    skm_sim_t coast_sim;
    skm_output_t drive = {SKM_OUTPUT_DRIVE, 0.5f, 0.5f};
    skm_output_t stop = {SKM_OUTPUT_BRAKE, 0.0f, 0.0f};
    unsigned int i;
    float brake_speed;
    float coast_speed;

    skm_sim_config_default(&config);
    skm_sim_init(&brake_sim, &config);
    skm_sim_init(&coast_sim, &config);
    skm_sim_output(&brake_sim, &drive);
    skm_sim_output(&coast_sim, &drive);
    for (i = 0U; i < 20U; ++i)
    {
        skm_sim_step(&brake_sim, 0.01f);
        skm_sim_step(&coast_sim, 0.01f);
    }

    skm_sim_output(&brake_sim, &stop);
    stop.mode = SKM_OUTPUT_COAST;
    skm_sim_output(&coast_sim, &stop);
    for (i = 0U; i < 10U; ++i)
    {
        skm_sim_step(&brake_sim, 0.01f);
        skm_sim_step(&coast_sim, 0.01f);
    }

    brake_speed = brake_sim.left_speed_mms;
    coast_speed = coast_sim.left_speed_mms;
    CHECK(brake_speed >= 0.0f);
    CHECK(coast_speed > brake_speed);
    CHECK(coast_sim.left_distance_mm > brake_sim.left_distance_mm);
    return true;
}

typedef bool (*test_function_t)(void);

typedef struct
{
    const char *name;
    test_function_t function;
} test_case_t;

int main(void)
{
    static const test_case_t tests[] = {
        {"configuration and idle output", test_configuration_and_idle_output},
        {"API busy, abort, and sensor fault", test_api_busy_abort_and_sensor_fault},
        {"wheel speed mode", test_wheel_speed_mode},
        {"straight distance and direction", test_straight_distance_and_direction},
        {"overshoot reported without reverse", test_overshoot_is_reported_without_reverse},
        {"learning save and restore", test_learning_can_be_saved_and_restored},
        {"brake learning convergence", test_brake_learning_convergence},
        {"abort does not learn", test_abort_does_not_learn},
        {"turn accuracy and hop spacing", test_turn_accuracy_and_hop_spacing},
        {"stuck turn reports stalled", test_stuck_turn_reports_stalled},
        {"reversed motor reports diverged", test_reversed_motor_reports_diverged},
        {"no-still gate blocks completion", test_no_still_gate_blocks_completion},
        {"heading trim reduces bias", test_heading_trim_reduces_systematic_bias},
        {"translation correction reduces drift", test_translation_correction_reduces_drift},
        {"deterministic replay", test_deterministic_replay},
        {"period independence", test_period_independence},
        {"timeout outputs brake", test_timeout_outputs_brake},
        {"brake and coast are distinct", test_brake_and_coast_are_distinct}
    };
    const size_t count = sizeof(tests) / sizeof(tests[0]);
    size_t i;
    unsigned int passed = 0U;

    for (i = 0U; i < count; ++i)
    {
        if (tests[i].function())
        {
            passed++;
            printf("[PASS] %s\n", tests[i].name);
        }
        else
        {
            printf("[FAIL] %s\n", tests[i].name);
        }
    }
    printf("%u/%u tests passed\n", passed, (unsigned int)count);
    return (passed == count) ? 0 : 1;
}
