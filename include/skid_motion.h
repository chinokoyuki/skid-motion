#ifndef SKID_MOTION_H
#define SKID_MOTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Portable skid-steer motion controller.
 *
 * Conventions:
 * - Distance is mm, speed is mm/s, acceleration is mm/s^2.
 * - Angles are deg and angular rates are deg/s.
 * - Time and brake-learning coefficients are seconds.
 * - Positive wheel speed moves the vehicle forward.
 * - Positive yaw and yaw rate mean counterclockwise, or a left turn.
 * - Left and right commands are normalized to [-1, +1].
 * - Positive differential (right > left) produces a left turn.
 *
 * The library has no hardware dependencies and performs no blocking waits.
 * One skm_tick call is one control period. Call skm_tick and the start/abort
 * functions from the same execution context; they are not thread-safe.
 */
typedef enum
{
    SKM_STOP_BRAKE = 0,
    SKM_STOP_COAST = 1
} skm_stop_mode_t;

typedef enum
{
    SKM_OUTPUT_BRAKE = 0,
    SKM_OUTPUT_COAST = 1,
    SKM_OUTPUT_DRIVE = 2
} skm_output_mode_t;

/*
 * Output requested from the platform adapter.
 *
 * BRAKE means active electrical braking. COAST means remove drive and allow
 * the wheels to turn freely. DRIVE uses left and right commands.
 */
typedef struct
{
    skm_output_mode_t mode;
    float left;
    float right;
} skm_output_t;

/* Called exactly once at the end of every skm_tick call. */
typedef void (*skm_output_fn)(void *user, const skm_output_t *output);

/*
 * Motion measurements for one control period.
 *
 * Deltas are the signed distance travelled since the previous tick. Yaw must
 * be continuous and must not wrap at +/-180 deg. Set valid to false when any
 * measurement is unavailable; an active action then stops with a sensor fault.
 */
typedef struct
{
    float left_delta_mm;
    float right_delta_mm;
    float left_speed_mms;
    float right_speed_mms;
    float yaw_deg;
    float yaw_rate_dps;
    bool valid;
} skm_sample_t;

typedef enum
{
    SKM_PHASE_IDLE = 0,
    SKM_PHASE_WHEEL_SPEED,
    SKM_PHASE_STRAIGHT_RUN,
    SKM_PHASE_STRAIGHT_SETTLE,
    SKM_PHASE_TURN_APPROACH,
    SKM_PHASE_TURN_SETTLE,
    SKM_PHASE_TURN_HOP,
    SKM_PHASE_TURN_HOP_SETTLE
} skm_phase_t;

typedef enum
{
    SKM_RESULT_NONE = 0,
    SKM_RESULT_RUNNING,
    SKM_RESULT_OK,
    SKM_RESULT_ABORTED,
    SKM_RESULT_TIMEOUT,
    SKM_RESULT_STALLED,
    SKM_RESULT_OVERSHOOT,
    SKM_RESULT_DIVERGED,
    SKM_RESULT_SENSOR_FAULT
} skm_result_t;

typedef enum
{
    SKM_API_OK = 0,
    SKM_API_BUSY,
    SKM_API_INVALID
} skm_api_result_t;

typedef struct
{
    float kp;
    float ki;
    float kd;
    float i_limit;
    float out_limit;
} skm_pid_config_t;

/*
 * Controller configuration.
 *
 * i_limit limits the integral state; zero disables integration. out_limit is
 * the absolute output limit. Defaults are a starting point only. Motor gains,
 * free speed, friction, and wheel geometry vary by platform, so speed_pi and
 * turn_pid normally require tuning on the target vehicle.
 */
typedef struct
{
    skm_pid_config_t speed_pi;
    skm_pid_config_t heading_pid;
    skm_pid_config_t turn_pid;

    float wheel_speed_max_mms;
    float cruise_mms;
    float accel_mms2;
    float position_kp;
    float position_tol_mm;

    float heading_corr_max_mms;
    float heading_trim_max_mms;
    float heading_trim_beta;
    float trim_min_time_s;
    float trim_cruise_ratio;

    float turn_stiction_mms;
    float turn_angle_tol_deg;
    float turn_small_angle_deg;
    float turn_diverge_margin_deg;

    float translation_kp;
    float translation_deadband_mm;
    float translation_corr_max_mms;

    float straight_timeout_s;
    float turn_timeout_s;
    float settle_min_s;
    float still_hold_s;
    float wheel_still_mms;
    float gyro_still_dps;

    float straight_brake_init_s;
    float straight_brake_min_s;
    float straight_brake_max_s;
    float straight_brake_alpha;
    float straight_brake_safety;
    float straight_learn_min_mms;

    float turn_brake_init_s;
    float turn_brake_min_s;
    float turn_brake_max_s;
    float turn_brake_alpha;
    float turn_brake_safety;
    float turn_learn_min_dps;

    float hop_t0_s;
    float hop_gain_init_dps;
    float hop_gain_min_dps;
    float hop_gain_max_dps;
    float hop_gain_alpha;
    float hop_level;
    float hop_understep;
    float hop_min_s;
    float hop_max_s;
    float hop_learn_min_s;
    float hop_retry_step_s;
    float hop_no_motion_deg;
    float hop_learn_min_deg;
    float hop_sample_max_deg;
    uint16_t hop_max_count;
    uint16_t hop_max_no_motion;
    uint16_t straight_max_approaches;
} skm_config_t;

typedef struct
{
    float straight_brake_s;
    float turn_brake_s;
    float heading_trim_mms;
    float hop_gain_dps;
} skm_learning_t;

typedef struct
{
    skm_phase_t phase;
    skm_result_t result;
    bool busy;
    float remaining_mm;
    float remaining_deg;
    float target_left_mms;
    float target_right_mms;
    uint16_t hop_count;
} skm_status_t;

typedef struct
{
    float integral;
    float output;
} skm_internal_pid_t;

/*
 * Controller context. Allocate statically or on the stack; no heap is used.
 * Fields after config are internal state and must not be modified by callers.
 */
typedef struct skm_s
{
    uint32_t magic;
    skm_config_t config;
    skm_output_fn output_fn;
    void *output_user;

    skm_phase_t phase;
    skm_result_t result;
    skm_stop_mode_t idle_stop;
    bool abort_pending;
    skm_stop_mode_t abort_stop;

    float action_time_s;
    float phase_time_s;
    float still_time_s;
    float distance_left_mm;
    float distance_right_mm;
    float yaw_start_deg;
    float yaw_target_deg;
    float straight_target_mm;
    float straight_v_cmd_mms;
    uint16_t straight_approaches;

    skm_internal_pid_t speed_left;
    skm_internal_pid_t speed_right;
    skm_internal_pid_t heading;
    skm_internal_pid_t turn;

    float trim_correction_sum;
    float trim_valid_time_s;

    bool brake_eligible;
    float brake_distance_mm;
    float brake_speed_mms;
    float brake_yaw_deg;
    float brake_rate_dps;

    float hop_planned_s;
    float hop_fired_s;
    float hop_direction;
    float hop_yaw_deg;
    float hop_error_deg;
    uint16_t hop_count;
    uint16_t hop_no_motion_count;
    bool hop_retry;

    float target_left_mms;
    float target_right_mms;
    float remaining_mm;
    float remaining_deg;
    skm_learning_t learned;
} skm_t;

/* Fill config with usable default values. */
void skm_config_default(skm_config_t *config);

/*
 * Initialize a controller. Returns SKM_API_INVALID for a null context,
 * invalid configuration, or missing output callback.
 */
skm_api_result_t skm_init(skm_t *ctx,
                          const skm_config_t *config,
                          skm_output_fn output_fn,
                          void *output_user);

/*
 * Start functions do not emit output or block. Output starts on the next tick.
 * They return SKM_API_BUSY without changing the active action if the
 * controller is not idle.
 */
skm_api_result_t skm_start_wheel_speed(skm_t *ctx,
                                       float left_mms,
                                       float right_mms);
skm_api_result_t skm_start_straight(skm_t *ctx, float distance_mm);
skm_api_result_t skm_start_turn(skm_t *ctx, float delta_deg);

/*
 * Request the next tick to stop with active braking or coasting. While idle,
 * the mode applies to subsequent idle ticks.
 */
void skm_abort(skm_t *ctx, skm_stop_mode_t stop_mode);

/*
 * Advance one control period. dt_s must be positive, finite, and no greater
 * than 1 second. Invalid measurements stop an active action with
 * SKM_RESULT_SENSOR_FAULT and request braking.
 */
void skm_tick(skm_t *ctx, const skm_sample_t *sample, float dt_s);

bool skm_is_busy(const skm_t *ctx);
void skm_get_status(const skm_t *ctx, skm_status_t *status);
void skm_get_learning(const skm_t *ctx, skm_learning_t *learning);

/*
 * Replace learned values while idle, for example after loading saved
 * calibration. Every value must be finite and within its configured range.
 */
skm_api_result_t skm_set_learning(skm_t *ctx, const skm_learning_t *learning);

/* Reset learned values while idle. Returns SKM_API_BUSY during an action. */
skm_api_result_t skm_reset_learning(skm_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* SKID_MOTION_H */
