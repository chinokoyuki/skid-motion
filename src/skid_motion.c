#include "skid_motion.h"

#include <math.h>
#include <string.h>

#include "skid_math.h"
#include "skid_pid.h"

#define SKM_MAGIC 0x534B4D31UL
#define SKM_EPSILON 1.0e-6f

static bool skm_valid_pid(const skm_pid_config_t *pid)
{
    return isfinite(pid->kp) && isfinite(pid->ki) && isfinite(pid->kd) &&
           isfinite(pid->i_limit) && isfinite(pid->out_limit) &&
           pid->kp >= 0.0f && pid->ki >= 0.0f && pid->kd >= 0.0f &&
           pid->i_limit >= 0.0f && pid->out_limit > 0.0f;
}

static bool skm_positive(float value)
{
    return isfinite(value) && value > 0.0f;
}

static bool skm_nonnegative(float value)
{
    return isfinite(value) && value >= 0.0f;
}

static bool skm_unit(float value)
{
    return isfinite(value) && value >= 0.0f && value <= 1.0f;
}

static bool skm_valid_config(const skm_config_t *c)
{
    if (!skm_valid_pid(&c->speed_pi) || !skm_valid_pid(&c->heading_pid) ||
        !skm_valid_pid(&c->turn_pid) || c->speed_pi.out_limit > 1.0f)
    {
        return false;
    }

    if (!skm_positive(c->wheel_speed_max_mms) || !skm_positive(c->cruise_mms) ||
        !skm_positive(c->accel_mms2) || !skm_positive(c->position_kp) ||
        !skm_positive(c->position_tol_mm) ||
        !skm_positive(c->heading_corr_max_mms) ||
        !skm_positive(c->heading_trim_max_mms) ||
        !skm_unit(c->heading_trim_beta) || !skm_positive(c->trim_min_time_s) ||
        !skm_unit(c->trim_cruise_ratio) ||
        c->cruise_mms + c->heading_corr_max_mms > c->wheel_speed_max_mms)
    {
        return false;
    }

    if (!skm_nonnegative(c->turn_stiction_mms) ||
        !skm_positive(c->turn_angle_tol_deg) ||
        !skm_positive(c->turn_small_angle_deg) ||
        !skm_positive(c->turn_diverge_margin_deg) ||
        c->turn_small_angle_deg < c->turn_angle_tol_deg ||
        c->turn_stiction_mms > c->turn_pid.out_limit)
    {
        return false;
    }

    if (!skm_nonnegative(c->translation_kp) ||
        !skm_nonnegative(c->translation_deadband_mm) ||
        !skm_nonnegative(c->translation_corr_max_mms) ||
        c->turn_pid.out_limit + c->translation_corr_max_mms >
            c->wheel_speed_max_mms)
    {
        return false;
    }

    if (!skm_positive(c->straight_timeout_s) || !skm_positive(c->turn_timeout_s) ||
        !skm_nonnegative(c->settle_min_s) || !skm_positive(c->still_hold_s) ||
        !skm_positive(c->wheel_still_mms) || !skm_positive(c->gyro_still_dps))
    {
        return false;
    }

    if (!skm_positive(c->straight_brake_min_s) ||
        !skm_positive(c->straight_brake_max_s) ||
        !isfinite(c->straight_brake_init_s) ||
        c->straight_brake_init_s < c->straight_brake_min_s ||
        c->straight_brake_init_s > c->straight_brake_max_s ||
        !skm_unit(c->straight_brake_alpha) ||
        !isfinite(c->straight_brake_safety) || c->straight_brake_safety < 1.0f ||
        !skm_positive(c->straight_learn_min_mms))
    {
        return false;
    }

    if (!skm_positive(c->turn_brake_min_s) || !skm_positive(c->turn_brake_max_s) ||
        !isfinite(c->turn_brake_init_s) ||
        c->turn_brake_init_s < c->turn_brake_min_s ||
        c->turn_brake_init_s > c->turn_brake_max_s ||
        !skm_unit(c->turn_brake_alpha) ||
        !isfinite(c->turn_brake_safety) || c->turn_brake_safety < 1.0f ||
        !skm_positive(c->turn_learn_min_dps))
    {
        return false;
    }

    if (!skm_nonnegative(c->hop_t0_s) || !skm_positive(c->hop_gain_min_dps) ||
        !skm_positive(c->hop_gain_max_dps) || !isfinite(c->hop_gain_init_dps) ||
        c->hop_gain_init_dps < c->hop_gain_min_dps ||
        c->hop_gain_init_dps > c->hop_gain_max_dps ||
        !skm_unit(c->hop_gain_alpha) || !skm_positive(c->hop_level) ||
        c->hop_level > 1.0f || !skm_unit(c->hop_understep) ||
        !skm_positive(c->hop_min_s) || !skm_positive(c->hop_max_s) ||
        c->hop_t0_s >= c->hop_min_s || c->hop_min_s > c->hop_max_s ||
        !skm_positive(c->hop_learn_min_s) ||
        c->hop_learn_min_s > c->hop_max_s ||
        !skm_positive(c->hop_retry_step_s) ||
        !skm_positive(c->hop_no_motion_deg) ||
        !skm_positive(c->hop_learn_min_deg) ||
        !skm_positive(c->hop_sample_max_deg) || c->hop_max_count == 0U ||
        c->hop_max_no_motion == 0U || c->straight_max_approaches == 0U)
    {
        return false;
    }

    return true;
}

static bool skm_valid_ctx(const skm_t *ctx)
{
    return ctx != NULL && ctx->magic == SKM_MAGIC;
}

static void skm_reset_controllers(skm_t *ctx)
{
    skm_pid_reset(&ctx->speed_left);
    skm_pid_reset(&ctx->speed_right);
    skm_pid_reset(&ctx->heading);
    skm_pid_reset(&ctx->turn);
}

static void skm_emit(skm_t *ctx, skm_output_mode_t mode, float left, float right)
{
    skm_output_t output;

    output.mode = mode;
    output.left = (mode == SKM_OUTPUT_DRIVE) ? skm_clampf(left, 1.0f) : 0.0f;
    output.right = (mode == SKM_OUTPUT_DRIVE) ? skm_clampf(right, 1.0f) : 0.0f;
    ctx->output_fn(ctx->output_user, &output);
}

static void skm_finish(skm_t *ctx, skm_result_t result)
{
    ctx->phase = SKM_PHASE_IDLE;
    ctx->result = result;
    ctx->idle_stop = SKM_STOP_BRAKE;
    ctx->target_left_mms = 0.0f;
    ctx->target_right_mms = 0.0f;
    ctx->abort_pending = false;
    skm_reset_controllers(ctx);
}

static void skm_enter_settle(skm_t *ctx, skm_phase_t phase)
{
    ctx->phase = phase;
    ctx->phase_time_s = 0.0f;
    ctx->still_time_s = 0.0f;
    ctx->target_left_mms = 0.0f;
    ctx->target_right_mms = 0.0f;
    skm_pid_reset(&ctx->speed_left);
    skm_pid_reset(&ctx->speed_right);
}

static void skm_start_common(skm_t *ctx, skm_phase_t phase)
{
    ctx->phase = phase;
    ctx->result = SKM_RESULT_RUNNING;
    ctx->action_time_s = 0.0f;
    ctx->phase_time_s = 0.0f;
    ctx->still_time_s = 0.0f;
    ctx->distance_left_mm = 0.0f;
    ctx->distance_right_mm = 0.0f;
    ctx->abort_pending = false;
    ctx->trim_correction_sum = 0.0f;
    ctx->trim_valid_time_s = 0.0f;
    ctx->brake_eligible = false;
    ctx->hop_count = 0U;
    ctx->hop_no_motion_count = 0U;
    ctx->hop_retry = false;
    ctx->target_left_mms = 0.0f;
    ctx->target_right_mms = 0.0f;
    ctx->remaining_mm = 0.0f;
    ctx->remaining_deg = 0.0f;
    skm_reset_controllers(ctx);
}

static bool skm_valid_sample(const skm_sample_t *sample, float dt_s)
{
    return sample != NULL && sample->valid && isfinite(dt_s) && dt_s > 0.0f &&
           dt_s <= 1.0f && isfinite(sample->left_delta_mm) &&
           isfinite(sample->right_delta_mm) && isfinite(sample->left_speed_mms) &&
           isfinite(sample->right_speed_mms) && isfinite(sample->yaw_deg) &&
           isfinite(sample->yaw_rate_dps);
}

static bool skm_update_still(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float wheel_speed = skm_maxf(fabsf(sample->left_speed_mms),
                                 fabsf(sample->right_speed_mms));

    if (wheel_speed <= ctx->config.wheel_still_mms &&
        fabsf(sample->yaw_rate_dps) <= ctx->config.gyro_still_dps)
    {
        ctx->still_time_s += dt_s;
    }
    else
    {
        ctx->still_time_s = 0.0f;
    }

    return ctx->phase_time_s + SKM_EPSILON >= ctx->config.settle_min_s &&
           ctx->still_time_s + SKM_EPSILON >= ctx->config.still_hold_s;
}

static void skm_drive_closed_loop(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float left = skm_pid_calculate(&ctx->speed_left, &ctx->config.speed_pi,
                                   ctx->target_left_mms, sample->left_speed_mms,
                                   0.0f, dt_s);
    float right = skm_pid_calculate(&ctx->speed_right, &ctx->config.speed_pi,
                                    ctx->target_right_mms, sample->right_speed_mms,
                                    0.0f, dt_s);
    skm_emit(ctx, SKM_OUTPUT_DRIVE, left, right);
}

static void skm_finish_straight(skm_t *ctx, skm_result_t result)
{
    if (result == SKM_RESULT_OK &&
        ctx->trim_valid_time_s + SKM_EPSILON >= ctx->config.trim_min_time_s)
    {
        float average = ctx->trim_correction_sum / ctx->trim_valid_time_s;
        float learned = ctx->learned.heading_trim_mms +
                        ctx->config.heading_trim_beta * average;
        ctx->learned.heading_trim_mms =
            skm_clampf(learned, ctx->config.heading_trim_max_mms);
    }
    skm_finish(ctx, result);
}

static void skm_tick_straight_run(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float direction = skm_signf(ctx->straight_target_mm);
    float distance = 0.5f * (ctx->distance_left_mm + ctx->distance_right_mm);
    float speed = 0.5f * (sample->left_speed_mms + sample->right_speed_mms);
    float remaining = ctx->straight_target_mm - distance;
    float remaining_dir = remaining * direction;
    float speed_dir = speed * direction;
    float desired = skm_clampf(ctx->config.position_kp * remaining_dir,
                               ctx->config.cruise_mms);
    float correction_pid;
    float correction;
    float prediction;
    bool normal_brake;

    ctx->remaining_mm = remaining;
    if (desired < 0.0f)
    {
        desired = 0.0f;
    }
    ctx->straight_v_cmd_mms = skm_move_toward(ctx->straight_v_cmd_mms, desired,
                                              ctx->config.accel_mms2 * dt_s);

    correction_pid = skm_pid_calculate(&ctx->heading, &ctx->config.heading_pid,
                                       ctx->yaw_start_deg, sample->yaw_deg,
                                       sample->yaw_rate_dps, dt_s);
    correction = skm_clampf(correction_pid + ctx->learned.heading_trim_mms,
                            ctx->config.heading_corr_max_mms);

    ctx->target_left_mms = direction * ctx->straight_v_cmd_mms - correction;
    ctx->target_right_mms = direction * ctx->straight_v_cmd_mms + correction;

    if (ctx->straight_v_cmd_mms >=
            ctx->config.trim_cruise_ratio * ctx->config.cruise_mms &&
        speed_dir >= ctx->config.trim_cruise_ratio * ctx->config.cruise_mms &&
        fabsf(correction_pid + ctx->learned.heading_trim_mms) <=
            ctx->config.heading_corr_max_mms &&
        fabsf(ctx->target_left_mms) <= ctx->config.wheel_speed_max_mms &&
        fabsf(ctx->target_right_mms) <= ctx->config.wheel_speed_max_mms &&
        fabsf(ctx->speed_left.output) < 1.0f && fabsf(ctx->speed_right.output) < 1.0f)
    {
        ctx->trim_correction_sum += correction_pid * dt_s;
        ctx->trim_valid_time_s += dt_s;
    }

    prediction = skm_maxf(speed_dir, 0.0f) *
                 ctx->config.straight_brake_safety *
                 ctx->learned.straight_brake_s;
    normal_brake = remaining_dir - prediction <= 0.5f * ctx->config.position_tol_mm;

    if (remaining_dir <= 0.0f || normal_brake)
    {
        ctx->brake_distance_mm = distance;
        ctx->brake_speed_mms = speed_dir;
        ctx->brake_eligible = normal_brake && remaining_dir > 0.0f &&
                              speed_dir >= ctx->config.straight_learn_min_mms;
        skm_enter_settle(ctx, SKM_PHASE_STRAIGHT_SETTLE);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    skm_drive_closed_loop(ctx, sample, dt_s);
}

static void skm_tick_straight_settle(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float direction = skm_signf(ctx->straight_target_mm);
    float distance = 0.5f * (ctx->distance_left_mm + ctx->distance_right_mm);
    float remaining;
    float remaining_dir;

    ctx->remaining_mm = ctx->straight_target_mm - distance;
    if (!skm_update_still(ctx, sample, dt_s))
    {
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    if (ctx->brake_eligible)
    {
        float slide = (distance - ctx->brake_distance_mm) * direction;
        float estimate = slide / ctx->brake_speed_mms;
        if (slide >= 0.0f && isfinite(estimate) &&
            estimate >= ctx->config.straight_brake_min_s &&
            estimate <= ctx->config.straight_brake_max_s)
        {
            float alpha = ctx->config.straight_brake_alpha;
            ctx->learned.straight_brake_s =
                (1.0f - alpha) * ctx->learned.straight_brake_s + alpha * estimate;
        }
        ctx->brake_eligible = false;
    }

    remaining = ctx->straight_target_mm - distance;
    remaining_dir = remaining * direction;
    ctx->remaining_mm = remaining;

    if (fabsf(remaining) <= ctx->config.position_tol_mm)
    {
        skm_finish_straight(ctx, SKM_RESULT_OK);
    }
    else if (remaining_dir < 0.0f)
    {
        skm_finish_straight(ctx, SKM_RESULT_OVERSHOOT);
    }
    else if (ctx->straight_approaches < ctx->config.straight_max_approaches)
    {
        ctx->straight_approaches++;
        ctx->phase = SKM_PHASE_STRAIGHT_RUN;
        ctx->phase_time_s = 0.0f;
        ctx->still_time_s = 0.0f;
        ctx->straight_v_cmd_mms = 0.0f;
        skm_reset_controllers(ctx);
    }
    else
    {
        skm_finish_straight(ctx, SKM_RESULT_STALLED);
    }

    skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
}

static bool skm_plan_hop(skm_t *ctx, float yaw_deg)
{
    float error = ctx->yaw_target_deg - yaw_deg;
    float pulse;

    ctx->remaining_deg = error;
    if (fabsf(error) <= ctx->config.turn_angle_tol_deg)
    {
        skm_finish(ctx, SKM_RESULT_OK);
        return false;
    }
    if (ctx->hop_count >= ctx->config.hop_max_count)
    {
        skm_finish(ctx, SKM_RESULT_STALLED);
        return false;
    }

    if (ctx->hop_retry)
    {
        pulse = ctx->hop_fired_s + ctx->config.hop_retry_step_s;
    }
    else
    {
        pulse = ctx->config.hop_t0_s + ctx->config.hop_understep * fabsf(error) /
                                           ctx->learned.hop_gain_dps;
    }
    if (pulse < ctx->config.hop_min_s)
    {
        pulse = ctx->config.hop_min_s;
    }
    if (pulse > ctx->config.hop_max_s)
    {
        pulse = ctx->config.hop_max_s;
    }

    ctx->hop_planned_s = pulse;
    ctx->hop_fired_s = 0.0f;
    ctx->hop_direction = skm_signf(error);
    ctx->hop_yaw_deg = yaw_deg;
    ctx->hop_error_deg = error;
    ctx->hop_count++;
    ctx->phase = SKM_PHASE_TURN_HOP;
    ctx->phase_time_s = 0.0f;
    ctx->still_time_s = 0.0f;
    return true;
}

static void skm_tick_turn_approach(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float error = ctx->yaw_target_deg - sample->yaw_deg;
    float translation = 0.5f * (ctx->distance_left_mm + ctx->distance_right_mm);
    float command = skm_pid_calculate(&ctx->turn, &ctx->config.turn_pid,
                                      ctx->yaw_target_deg, sample->yaw_deg,
                                      sample->yaw_rate_dps, dt_s);
    float common = skm_clampf(-ctx->config.translation_kp * translation,
                              ctx->config.translation_corr_max_mms);
    float closing_rate;
    float predicted;

    ctx->remaining_deg = error;
    if (fabsf(translation) < ctx->config.translation_deadband_mm)
    {
        common = 0.0f;
    }
    if (fabsf(error) > ctx->config.turn_angle_tol_deg && command * error > 0.0f &&
        fabsf(command) < ctx->config.turn_stiction_mms)
    {
        command = skm_signf(error) * ctx->config.turn_stiction_mms;
    }

    ctx->target_left_mms = -command + common;
    ctx->target_right_mms = command + common;

    if (ctx->action_time_s > 1.5f &&
        fabsf(error) > fabsf(ctx->yaw_target_deg - ctx->yaw_start_deg) +
                           ctx->config.turn_diverge_margin_deg)
    {
        skm_finish(ctx, SKM_RESULT_DIVERGED);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    closing_rate = skm_signf(error) * sample->yaw_rate_dps;
    predicted = fabsf(error) - skm_maxf(closing_rate, 0.0f) *
                                  ctx->config.turn_brake_safety *
                                  ctx->learned.turn_brake_s;
    if (fabsf(error) <= 0.6f * ctx->config.turn_angle_tol_deg ||
        (closing_rate > 0.0f &&
         predicted <= 0.6f * ctx->config.turn_angle_tol_deg))
    {
        ctx->brake_yaw_deg = sample->yaw_deg;
        ctx->brake_rate_dps = sample->yaw_rate_dps;
        ctx->brake_eligible = fabsf(error) > 0.6f * ctx->config.turn_angle_tol_deg &&
                              error * sample->yaw_rate_dps > 0.0f &&
                              fabsf(sample->yaw_rate_dps) >=
                                  ctx->config.turn_learn_min_dps;
        skm_enter_settle(ctx, SKM_PHASE_TURN_SETTLE);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    skm_drive_closed_loop(ctx, sample, dt_s);
}

static void skm_tick_turn_settle(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float error = ctx->yaw_target_deg - sample->yaw_deg;
    ctx->remaining_deg = error;

    if (!skm_update_still(ctx, sample, dt_s))
    {
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    if (ctx->brake_eligible)
    {
        float slide = (sample->yaw_deg - ctx->brake_yaw_deg) *
                      skm_signf(ctx->brake_rate_dps);
        float estimate = slide / fabsf(ctx->brake_rate_dps);
        if (slide >= 0.0f && isfinite(estimate) &&
            estimate >= ctx->config.turn_brake_min_s &&
            estimate <= ctx->config.turn_brake_max_s)
        {
            float alpha = ctx->config.turn_brake_alpha;
            ctx->learned.turn_brake_s =
                (1.0f - alpha) * ctx->learned.turn_brake_s + alpha * estimate;
        }
        ctx->brake_eligible = false;
    }

    if (fabsf(error) <= ctx->config.turn_angle_tol_deg)
    {
        skm_finish(ctx, SKM_RESULT_OK);
    }
    else
    {
        ctx->hop_retry = false;
        skm_plan_hop(ctx, sample->yaw_deg);
    }
    skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
}

static void skm_tick_turn_hop(skm_t *ctx, float dt_s)
{
    if (ctx->hop_fired_s + SKM_EPSILON >= ctx->hop_planned_s)
    {
        skm_enter_settle(ctx, SKM_PHASE_TURN_HOP_SETTLE);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    ctx->target_left_mms = 0.0f;
    ctx->target_right_mms = 0.0f;
    skm_emit(ctx, SKM_OUTPUT_DRIVE, -ctx->hop_direction * ctx->config.hop_level,
             ctx->hop_direction * ctx->config.hop_level);
    ctx->hop_fired_s += dt_s;
}

static void skm_tick_hop_settle(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    float error = ctx->yaw_target_deg - sample->yaw_deg;
    float rotated;
    ctx->remaining_deg = error;

    if (!skm_update_still(ctx, sample, dt_s))
    {
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    rotated = ctx->hop_direction * (sample->yaw_deg - ctx->hop_yaw_deg);
    if (rotated < -ctx->config.hop_no_motion_deg ||
        fabsf(error) > fabsf(ctx->hop_error_deg) + ctx->config.turn_diverge_margin_deg)
    {
        skm_finish(ctx, SKM_RESULT_DIVERGED);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    if (rotated <= ctx->config.hop_no_motion_deg)
    {
        ctx->hop_no_motion_count++;
        if (ctx->hop_no_motion_count > ctx->config.hop_max_no_motion)
        {
            skm_finish(ctx, SKM_RESULT_STALLED);
        }
        else
        {
            ctx->hop_retry = true;
            skm_plan_hop(ctx, sample->yaw_deg);
        }
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    if (ctx->hop_fired_s >= ctx->config.hop_learn_min_s &&
        rotated >= ctx->config.hop_learn_min_deg &&
        rotated <= ctx->config.hop_sample_max_deg &&
        ctx->hop_fired_s > ctx->config.hop_t0_s)
    {
        float estimate = rotated / (ctx->hop_fired_s - ctx->config.hop_t0_s);
        if (isfinite(estimate) && estimate >= ctx->config.hop_gain_min_dps &&
            estimate <= ctx->config.hop_gain_max_dps)
        {
            float alpha = ctx->config.hop_gain_alpha;
            ctx->learned.hop_gain_dps =
                (1.0f - alpha) * ctx->learned.hop_gain_dps + alpha * estimate;
        }
    }

    ctx->hop_no_motion_count = 0U;
    if (fabsf(error) <= ctx->config.turn_angle_tol_deg)
    {
        skm_finish(ctx, SKM_RESULT_OK);
    }
    else
    {
        ctx->hop_retry = false;
        skm_plan_hop(ctx, sample->yaw_deg);
    }
    skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
}

static void skm_set_pid_default(skm_pid_config_t *pid, float kp, float ki,
                                float kd, float i_limit, float out_limit)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->i_limit = i_limit;
    pid->out_limit = out_limit;
}

void skm_config_default(skm_config_t *c)
{
    if (c == NULL)
    {
        return;
    }
    memset(c, 0, sizeof(*c));

    skm_set_pid_default(&c->speed_pi, 0.020f, 0.020f, 0.0f, 100.0f, 1.0f);
    skm_set_pid_default(&c->heading_pid, 4.0f, 0.5f, 0.05f, 80.0f, 80.0f);
    skm_set_pid_default(&c->turn_pid, 6.0f, 0.0f, 1.2f, 0.0f, 110.0f);

    c->wheel_speed_max_mms = 250.0f;
    c->cruise_mms = 100.0f;
    c->accel_mms2 = 1200.0f;
    c->position_kp = 4.0f;
    c->position_tol_mm = 2.0f;
    c->heading_corr_max_mms = 80.0f;
    c->heading_trim_max_mms = 80.0f;
    c->heading_trim_beta = 0.8f;
    c->trim_min_time_s = 0.20f;
    c->trim_cruise_ratio = 0.70f;

    c->turn_stiction_mms = 45.0f;
    c->turn_angle_tol_deg = 1.0f;
    c->turn_small_angle_deg = 5.0f;
    c->turn_diverge_margin_deg = 30.0f;
    c->translation_kp = 2.0f;
    c->translation_deadband_mm = 0.5f;
    c->translation_corr_max_mms = 40.0f;

    c->straight_timeout_s = 10.0f;
    c->turn_timeout_s = 15.0f;
    c->settle_min_s = 0.05f;
    c->still_hold_s = 0.03f;
    c->wheel_still_mms = 5.0f;
    c->gyro_still_dps = 2.0f;

    c->straight_brake_init_s = 0.06f;
    c->straight_brake_min_s = 0.005f;
    c->straight_brake_max_s = 0.40f;
    c->straight_brake_alpha = 0.40f;
    c->straight_brake_safety = 1.0f;
    c->straight_learn_min_mms = 30.0f;

    c->turn_brake_init_s = 0.08f;
    c->turn_brake_min_s = 0.005f;
    c->turn_brake_max_s = 0.25f;
    c->turn_brake_alpha = 0.40f;
    c->turn_brake_safety = 1.0f;
    c->turn_learn_min_dps = 10.0f;

    c->hop_t0_s = 0.012f;
    c->hop_gain_init_dps = 100.0f;
    c->hop_gain_min_dps = 20.0f;
    c->hop_gain_max_dps = 400.0f;
    c->hop_gain_alpha = 0.50f;
    c->hop_level = 0.42f;
    c->hop_understep = 0.85f;
    c->hop_min_s = 0.020f;
    c->hop_max_s = 0.120f;
    c->hop_learn_min_s = 0.025f;
    c->hop_retry_step_s = 0.010f;
    c->hop_no_motion_deg = 0.05f;
    c->hop_learn_min_deg = 0.15f;
    c->hop_sample_max_deg = 10.0f;
    c->hop_max_count = 12U;
    c->hop_max_no_motion = 3U;
    c->straight_max_approaches = 3U;
}

skm_api_result_t skm_init(skm_t *ctx, const skm_config_t *config,
                          skm_output_fn output_fn, void *output_user)
{
    skm_config_t copy;

    if (ctx == NULL || config == NULL || output_fn == NULL || !skm_valid_config(config))
    {
        return SKM_API_INVALID;
    }
    copy = *config;
    memset(ctx, 0, sizeof(*ctx));
    ctx->config = copy;
    ctx->output_fn = output_fn;
    ctx->output_user = output_user;
    ctx->phase = SKM_PHASE_IDLE;
    ctx->result = SKM_RESULT_NONE;
    ctx->idle_stop = SKM_STOP_BRAKE;
    ctx->learned.straight_brake_s = copy.straight_brake_init_s;
    ctx->learned.turn_brake_s = copy.turn_brake_init_s;
    ctx->learned.hop_gain_dps = copy.hop_gain_init_dps;
    skm_reset_controllers(ctx);
    ctx->magic = SKM_MAGIC;
    return SKM_API_OK;
}

skm_api_result_t skm_start_wheel_speed(skm_t *ctx, float left_mms, float right_mms)
{
    if (!skm_valid_ctx(ctx))
    {
        return SKM_API_INVALID;
    }
    if (ctx->phase != SKM_PHASE_IDLE)
    {
        return SKM_API_BUSY;
    }
    if (!isfinite(left_mms) || !isfinite(right_mms) ||
        fabsf(left_mms) > ctx->config.wheel_speed_max_mms ||
        fabsf(right_mms) > ctx->config.wheel_speed_max_mms)
    {
        return SKM_API_INVALID;
    }
    skm_start_common(ctx, SKM_PHASE_WHEEL_SPEED);
    ctx->target_left_mms = left_mms;
    ctx->target_right_mms = right_mms;
    return SKM_API_OK;
}

skm_api_result_t skm_start_straight(skm_t *ctx, float distance_mm)
{
    if (!skm_valid_ctx(ctx))
    {
        return SKM_API_INVALID;
    }
    if (ctx->phase != SKM_PHASE_IDLE)
    {
        return SKM_API_BUSY;
    }
    if (!isfinite(distance_mm) || fabsf(distance_mm) < SKM_EPSILON)
    {
        return SKM_API_INVALID;
    }
    skm_start_common(ctx, SKM_PHASE_STRAIGHT_RUN);
    ctx->straight_target_mm = distance_mm;
    ctx->remaining_mm = distance_mm;
    ctx->straight_v_cmd_mms = 0.0f;
    ctx->straight_approaches = 1U;
    return SKM_API_OK;
}

skm_api_result_t skm_start_turn(skm_t *ctx, float delta_deg)
{
    if (!skm_valid_ctx(ctx))
    {
        return SKM_API_INVALID;
    }
    if (ctx->phase != SKM_PHASE_IDLE)
    {
        return SKM_API_BUSY;
    }
    if (!isfinite(delta_deg) || fabsf(delta_deg) < SKM_EPSILON)
    {
        return SKM_API_INVALID;
    }
    skm_start_common(ctx, (fabsf(delta_deg) < ctx->config.turn_small_angle_deg)
                              ? SKM_PHASE_TURN_SETTLE
                              : SKM_PHASE_TURN_APPROACH);
    ctx->yaw_target_deg = delta_deg;
    ctx->remaining_deg = delta_deg;
    return SKM_API_OK;
}

void skm_abort(skm_t *ctx, skm_stop_mode_t stop_mode)
{
    if (!skm_valid_ctx(ctx) ||
        (stop_mode != SKM_STOP_BRAKE && stop_mode != SKM_STOP_COAST))
    {
        return;
    }
    if (ctx->phase == SKM_PHASE_IDLE)
    {
        ctx->idle_stop = stop_mode;
    }
    else
    {
        ctx->abort_pending = true;
        ctx->abort_stop = stop_mode;
    }
}

void skm_tick(skm_t *ctx, const skm_sample_t *sample, float dt_s)
{
    if (!skm_valid_ctx(ctx))
    {
        return;
    }

    if (ctx->phase == SKM_PHASE_IDLE)
    {
        skm_emit(ctx, (ctx->idle_stop == SKM_STOP_COAST) ? SKM_OUTPUT_COAST
                                                         : SKM_OUTPUT_BRAKE,
                 0.0f, 0.0f);
        return;
    }

    if (ctx->abort_pending)
    {
        skm_stop_mode_t mode = ctx->abort_stop;
        skm_finish(ctx, SKM_RESULT_ABORTED);
        ctx->idle_stop = mode;
        skm_emit(ctx, (mode == SKM_STOP_COAST) ? SKM_OUTPUT_COAST : SKM_OUTPUT_BRAKE,
                 0.0f, 0.0f);
        return;
    }

    if (!skm_valid_sample(sample, dt_s))
    {
        skm_finish(ctx, SKM_RESULT_SENSOR_FAULT);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    if (ctx->action_time_s == 0.0f &&
        (ctx->phase == SKM_PHASE_STRAIGHT_RUN || ctx->phase == SKM_PHASE_TURN_APPROACH ||
         ctx->phase == SKM_PHASE_TURN_SETTLE))
    {
        ctx->yaw_start_deg = sample->yaw_deg;
        if (ctx->phase != SKM_PHASE_STRAIGHT_RUN)
        {
            ctx->yaw_target_deg += sample->yaw_deg;
        }
    }

    ctx->distance_left_mm += sample->left_delta_mm;
    ctx->distance_right_mm += sample->right_delta_mm;
    ctx->action_time_s += dt_s;
    ctx->phase_time_s += dt_s;

    if ((ctx->phase == SKM_PHASE_STRAIGHT_RUN ||
         ctx->phase == SKM_PHASE_STRAIGHT_SETTLE) &&
        ctx->action_time_s > ctx->config.straight_timeout_s)
    {
        skm_finish(ctx, SKM_RESULT_TIMEOUT);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }
    if (ctx->phase != SKM_PHASE_WHEEL_SPEED && ctx->phase != SKM_PHASE_STRAIGHT_RUN &&
        ctx->phase != SKM_PHASE_STRAIGHT_SETTLE &&
        ctx->action_time_s > ctx->config.turn_timeout_s)
    {
        skm_finish(ctx, SKM_RESULT_TIMEOUT);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        return;
    }

    switch (ctx->phase)
    {
    case SKM_PHASE_WHEEL_SPEED:
        skm_drive_closed_loop(ctx, sample, dt_s);
        break;
    case SKM_PHASE_STRAIGHT_RUN:
        skm_tick_straight_run(ctx, sample, dt_s);
        break;
    case SKM_PHASE_STRAIGHT_SETTLE:
        skm_tick_straight_settle(ctx, sample, dt_s);
        break;
    case SKM_PHASE_TURN_APPROACH:
        skm_tick_turn_approach(ctx, sample, dt_s);
        break;
    case SKM_PHASE_TURN_SETTLE:
        skm_tick_turn_settle(ctx, sample, dt_s);
        break;
    case SKM_PHASE_TURN_HOP:
        skm_tick_turn_hop(ctx, dt_s);
        break;
    case SKM_PHASE_TURN_HOP_SETTLE:
        skm_tick_hop_settle(ctx, sample, dt_s);
        break;
    default:
        skm_finish(ctx, SKM_RESULT_ABORTED);
        skm_emit(ctx, SKM_OUTPUT_BRAKE, 0.0f, 0.0f);
        break;
    }
}

bool skm_is_busy(const skm_t *ctx)
{
    return skm_valid_ctx(ctx) && ctx->phase != SKM_PHASE_IDLE;
}

void skm_get_status(const skm_t *ctx, skm_status_t *status)
{
    if (status == NULL)
    {
        return;
    }
    memset(status, 0, sizeof(*status));
    if (!skm_valid_ctx(ctx))
    {
        return;
    }
    status->phase = ctx->phase;
    status->result = ctx->result;
    status->busy = ctx->phase != SKM_PHASE_IDLE;
    status->remaining_mm = ctx->remaining_mm;
    status->remaining_deg = ctx->remaining_deg;
    status->target_left_mms = ctx->target_left_mms;
    status->target_right_mms = ctx->target_right_mms;
    status->hop_count = ctx->hop_count;
}

void skm_get_learning(const skm_t *ctx, skm_learning_t *learning)
{
    if (learning == NULL)
    {
        return;
    }
    memset(learning, 0, sizeof(*learning));
    if (skm_valid_ctx(ctx))
    {
        *learning = ctx->learned;
    }
}

skm_api_result_t skm_set_learning(skm_t *ctx, const skm_learning_t *learning)
{
    if (!skm_valid_ctx(ctx))
    {
        return SKM_API_INVALID;
    }
    if (ctx->phase != SKM_PHASE_IDLE)
    {
        return SKM_API_BUSY;
    }
    if (learning == NULL || !isfinite(learning->straight_brake_s) ||
        !isfinite(learning->turn_brake_s) || !isfinite(learning->heading_trim_mms) ||
        !isfinite(learning->hop_gain_dps) ||
        learning->straight_brake_s < ctx->config.straight_brake_min_s ||
        learning->straight_brake_s > ctx->config.straight_brake_max_s ||
        learning->turn_brake_s < ctx->config.turn_brake_min_s ||
        learning->turn_brake_s > ctx->config.turn_brake_max_s ||
        fabsf(learning->heading_trim_mms) > ctx->config.heading_trim_max_mms ||
        learning->hop_gain_dps < ctx->config.hop_gain_min_dps ||
        learning->hop_gain_dps > ctx->config.hop_gain_max_dps)
    {
        return SKM_API_INVALID;
    }
    ctx->learned = *learning;
    return SKM_API_OK;
}

skm_api_result_t skm_reset_learning(skm_t *ctx)
{
    if (!skm_valid_ctx(ctx))
    {
        return SKM_API_INVALID;
    }
    if (ctx->phase != SKM_PHASE_IDLE)
    {
        return SKM_API_BUSY;
    }
    ctx->learned.straight_brake_s = ctx->config.straight_brake_init_s;
    ctx->learned.turn_brake_s = ctx->config.turn_brake_init_s;
    ctx->learned.heading_trim_mms = 0.0f;
    ctx->learned.hop_gain_dps = ctx->config.hop_gain_init_dps;
    return SKM_API_OK;
}
