# skid-motion

[简体中文](README.zh-CN.md)

A portable C99 motion controller for four-wheel skid-steer platforms. It provides a wheel-speed loop, distance and heading control, predictive braking, online parameter learning, and precise in-place turns. Hardware access is supplied through one output callback and one periodic sensor sample.

The core library has no dependence on STM32 HAL, an RTOS, or a specific motor driver. A deterministic host-side vehicle simulator is included for examples and tests.

## What is included

- Signed straight-line distance control with encoder feedback
- IMU heading hold with acceleration ramps
- Predictive braking with an online slide estimate
- Cross-segment heading trim for repeatable left/right bias
- Two-stage in-place turning: continuous approach, brake, then short open-loop pulses
- Stillness-gated measurement after braking
- Online hop-gain and brake-time learning
- Common-mode correction for translation during in-place turns
- Explicit brake, coast, abort, timeout, overshoot, stall, and divergence reporting
- Non-blocking `skm_tick()` execution
- Host simulator with configurable asymmetry and fault injection

The library does not include motor drivers, encoder drivers, IMU drivers, PID autotuning, obstacle detection, path planning, or heading-grid assumptions.

## Control structure

```text
encoder + IMU sample
        |
        v
   skm_tick(sample, dt)
        |
        +--> wheel-speed PI --> normalized left/right output callback
        |
        +--> straight: distance loop + heading loop + predicted brake
        |
        +--> turn: angle loop + predicted brake + pulse approach
```

One call to `skm_tick()` advances one control period. Start and abort calls only change requests; they do not block or write hardware. The output callback is called once at the end of every tick.

## Conventions

| Quantity | Unit and sign |
|---|---|
| Distance, wheel speed, acceleration | mm, mm/s, mm/s² |
| Angle, yaw rate | deg, deg/s |
| Time, brake coefficient | seconds |
| Positive wheel speed | moves the vehicle forward |
| Positive yaw and yaw rate | counterclockwise, left turn |
| Positive turn target | left turn |
| Motor output | normalized `[-1, +1]` |
| Positive differential (`right - left`) | left turn |

Yaw must be continuous. Do not wrap the sample at ±180°.

## Build and test

Requires CMake 3.15 or newer and a C99 compiler.

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

Run the simulator example:

```sh
./build/skid_simulate
```

On Windows with a multi-configuration generator, the executable may be under `build/Debug/`.

To install and then consume the package:

```sh
cmake --install build --prefix /path/to/prefix
```

```cmake
find_package(skid_motion CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE skid::motion skid::sim)
```

Use `-DSKM_BUILD_SIM=OFF` to build only the controller library. `SKM_ENABLE_SANITIZERS=ON` enables ASan and UBSan for GCC or Clang builds.

The test suite covers API semantics, speed control, forward and reverse distance, overshoot, brake-time learning, learning rejection after abort and sensor faults, turn accuracy, pulse spacing, stall and divergence detection, stillness gating, heading trim, translation correction, deterministic replay, period independence, timeout behavior, and distinct brake/coast responses.

## Integration

### 1. Implement the output callback

```c
static void motor_output(void *user, const skm_output_t *output)
{
    (void)user;

    if (output->mode == SKM_OUTPUT_BRAKE) {
        motor_brake_all();
    } else if (output->mode == SKM_OUTPUT_COAST) {
        motor_coast_all();
    } else {
        motor_left(output->left);
        motor_right(output->right);
    }
}
```

`BRAKE` should use the strongest electrical braking the platform supports. If the driver cannot brake actively, handle that as a platform limitation rather than silently treating it as `COAST`.

### 2. Initialize the controller

```c
skm_config_t config;
skm_t motion;

skm_config_default(&config);
/* Adjust gains and limits for the vehicle. */
if (skm_init(&motion, &config, motor_output, NULL) != SKM_API_OK) {
    /* Handle invalid configuration. */
}
```

`skm_t` is allocated by the caller. The library uses no heap.

### 3. Provide periodic samples

```c
void control_tick(void)
{
    skm_sample_t sample;

    sample.left_delta_mm   = read_left_delta_mm();
    sample.right_delta_mm  = read_right_delta_mm();
    sample.left_speed_mms  = read_left_speed_mms();
    sample.right_speed_mms = read_right_speed_mms();
    sample.yaw_deg         = read_continuous_yaw_deg();
    sample.yaw_rate_dps    = read_yaw_rate_dps();
    sample.valid           = sensors_ok();

    skm_tick(&motion, &sample, 0.001f);
}
```

Start an action from the same execution context:

```c
if (skm_start_straight(&motion, 500.0f) == SKM_API_OK) {
    /* Action accepted; output begins on the next tick. */
}
```

The start functions return `SKM_API_BUSY` while another action is active. The active action is not modified in that case. Wheel-speed mode remains active until `skm_abort()` is called.

## API summary

| Function | Purpose |
|---|---|
| `skm_config_default()` | Fill a configuration with default values |
| `skm_init()` | Validate configuration and initialize the context |
| `skm_start_wheel_speed()` | Run the wheel-speed loop with fixed left/right targets |
| `skm_start_straight()` | Move a signed distance while holding heading |
| `skm_start_turn()` | Turn in place by a signed angle |
| `skm_abort()` | Request brake or coast on the next tick |
| `skm_tick()` | Advance one control period |
| `skm_is_busy()` | Return whether an action is active |
| `skm_get_status()` | Read phase, result, remaining error, and targets |
| `skm_get_learning()` | Read learned brake times, trim, and hop gain |
| `skm_set_learning()` | Restore saved calibration while idle |
| `skm_reset_learning()` | Restore learned values to configuration defaults while idle |

Action results:

| Result | Meaning |
|---|---|
| `SKM_RESULT_OK` | Final error is within tolerance |
| `SKM_RESULT_ABORTED` | Stopped by `skm_abort()` |
| `SKM_RESULT_TIMEOUT` | Action time limit exceeded |
| `SKM_RESULT_STALLED` | Motion did not make enough progress |
| `SKM_RESULT_OVERSHOOT` | Final position is beyond tolerance; no reverse correction is attempted |
| `SKM_RESULT_DIVERGED` | Feedback moved in the wrong direction |
| `SKM_RESULT_SENSOR_FAULT` | Sample was invalid, non-finite, or `dt_s` was outside the accepted range |

## Control behavior

### Straight motion

The distance loop produces a forward speed command. The command is rate-limited by `accel_mms2`. The heading loop adds a differential correction, and the wheel-speed loop tracks the resulting left and right targets.

Braking starts when the predicted slide would reach the target:

```text
slide = forward_speed * brake_safety * learned_brake_time
brake when remaining_distance - slide <= 0.5 * position_tolerance
```

After stopping, the controller waits for a continuous stillness interval, measures the actual slide, and updates the brake estimate only if the sample is valid and within configured limits. An under-traveling action makes another forward approach; it never reverses to correct a small overshoot.

### Heading trim

During stable cruise, the heading PID contribution is accumulated over time. A successful straight action commits a fraction of the average correction into a feed-forward trim. Saturated outputs, low speed, aborted actions, and failed actions do not contribute.

This reduces repeatable bias caused by unequal motor gains, wheel friction, or drivetrain losses. It does not replace mechanical alignment or correct a reversed sensor sign.

### In-place turns

Large turns use a continuous angle controller with static-friction feed-forward. Braking is predicted only when the current rotation is reducing the angle error. After the vehicle is still, short open-loop pulses approach the target.

Each pulse is followed by braking and a full stillness check. The measured rotation updates the affine pulse model only when the rotation is in the commanded direction and inside the accepted range. A pulse that produces no motion increases the next pulse up to a hard limit; repeated failures return `STALLED`.

Small turns skip the continuous approach and begin with pulse control.

### Learning limits

Learned values are stored in `skm_t`, not in persistent memory. Save `skm_get_learning()` externally and restore it with `skm_set_learning()` if the platform needs calibration to survive a reset. Learned values are not updated after aborts, timeouts, sensor faults, wrong-direction motion, or samples outside the configured physical ranges.

## Key configuration

| Field | Role |
|---|---|
| `speed_pi` | Wheel-speed loop gains and normalized output limit |
| `heading_pid` | Straight-line heading correction |
| `turn_pid` | Continuous turn approach |
| `cruise_mms`, `accel_mms2` | Straight speed and command ramp |
| `position_kp`, `position_tol_mm` | Distance response and accepted error |
| `heading_corr_max_mms` | Maximum differential correction |
| `heading_trim_beta` | Fraction of average cruise correction committed to trim |
| `turn_stiction_mms` | Minimum useful turn command near the target |
| `turn_angle_tol_deg` | In-place turn tolerance |
| `straight_brake_*`, `turn_brake_*` | Initial, range, and learning rate for brake estimates |
| `hop_*` | Pulse timing, gain range, level, retry, and safety limits |
| `*_timeout_s` | Hard action duration limits |

The defaults are generic starting points. Tune in this order:

1. Verify motor and IMU polarity with the wheels raised.
2. Tune `speed_pi` under normal load.
3. Tune `heading_pid` and confirm straight motion on a level surface.
4. Check brake prediction and learning at the intended cruise speed.
5. Tune the turn approach, then pulse level and limits.
6. Test fault behavior with invalid samples, a raised vehicle, and a blocked wheel.

## Simulator

`skid_sim` is a deterministic first-order model with independent left/right gains, static friction, separate drive/brake/coast response, and optional faults.

```c
skm_sim_config_t sim_config;
skm_sim_t sim;
skm_t motion;

skm_sim_config_default(&sim_config);
skm_sim_init(&sim, &sim_config);
skm_init(&motion, &config, skm_sim_output, &sim);
```

The simulator supports:

- Unequal left/right gains
- Constant common-mode disturbance
- Stuck wheels
- Reversed motor or gyro sign
- Forced yaw rate for stillness-gate tests
- Invalid samples

It does not model wheel slip, suspension, tire deformation, encoder quantization, or real sensor noise. Passing simulation tests does not establish performance on a physical vehicle.

## Limitations and safety

- The controller assumes a four-wheel skid-steer platform; it is not a mecanum or Ackermann controller.
- The heading loop requires a continuous yaw estimate with the documented sign.
- Outputs are normalized. The adapter must map them to safe motor commands and enforce its own current or torque limits.
- `dt_s` must be positive and no greater than 1 second. Normal operation should use a fixed control period.
- API calls and `skm_tick()` are not thread-safe.
- `skm_abort()` takes effect on the next tick. It is not a hardware emergency stop.
- Use an independent hardware emergency stop for motor and actuator power. Do not rely on software timing for personnel safety.
- Validate behavior with the vehicle lifted before ground testing.

## License

MIT. See [LICENSE](LICENSE).
