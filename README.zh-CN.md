# skid-motion

[English](README.md)

一个用于四轮滑移转向平台的便携 C99 运动控制库。它包含轮速内环、里程与航向控制、预测刹车、在线参数学习和原地精确转弯。硬件访问通过一个输出回调和一份周期传感器采样接入。

核心库不依赖 STM32 HAL、RTOS 或特定电机驱动。仓库包含一个确定性的主机端车辆仿真器，用于示例和测试。

## 功能范围

- 带符号的编码器里程直行控制
- IMU 航向保持与加速度斜坡
- 带在线滑行估计的预测刹车
- 跨直行段的航向自动配平，抑制重复性左右偏差
- 两段式原地转弯：连续粗调、刹车，再短脉冲逼近
- 刹车后的连续停稳确认
- 点动增益和刹车时间的在线学习
- 原地转弯时的平移共模纠偏
- 明确的刹车、滑行、中止、超时、过冲、卡滞和发散结果
- 非阻塞的 `skm_tick()` 执行
- 支持左右不对称和故障注入的主机仿真器

本库不包含电机驱动、编码器驱动、IMU 驱动、PID 自动整定、障碍物检测、路径规划或航向网格假设。

## 控制结构

```text
编码器 + IMU 采样
        |
        v
   skm_tick(sample, dt)
        |
        +--> 轮速 PI --> 归一化左右输出回调
        |
        +--> 直行：里程环 + 航向环 + 预测刹车
        |
        +--> 转弯：角度环 + 预测刹车 + 脉冲逼近
```

每次 `skm_tick()` 推进一个控制周期。启动和中止接口只修改请求，不会阻塞或直接操作硬件。输出回调在每次 tick 末尾调用一次。

## 约定

| 物理量 | 单位与符号 |
|---|---|
| 距离、轮速、加速度 | mm、mm/s、mm/s² |
| 角度、角速度 | deg、deg/s |
| 时间、刹车系数 | 秒 |
| 正轮速 | 使车辆前进 |
| 正 yaw 与正角速度 | 逆时针，左转 |
| 正转弯目标 | 左转 |
| 电机输出 | 归一化 `[-1, +1]` |
| 正差速（右减左） | 左转 |

yaw 必须连续，传感器采样不能在 ±180° 处回绕。

## 构建与测试

需要 CMake 3.15 或更新版本，以及支持 C99 的编译器。

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

运行仿真示例：

```sh
./build/skid_simulate
```

在 Windows 多配置生成器（如 Visual Studio）下，可执行文件也可能位于 `build/Debug/`。

安装后使用：

```sh
cmake --install build --prefix /path/to/prefix
```

```cmake
find_package(skid_motion CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE skid::motion skid::sim)
```

使用 `-DSKM_BUILD_SIM=OFF` 可以只构建控制器库。`SKM_ENABLE_SANITIZERS=ON` 为 GCC 或 Clang 构建启用 ASan 与 UBSan。

测试覆盖 API 语义、轮速控制、前进与后退里程、过冲、刹车时间学习、中止和传感器故障后的学习冻结、转弯精度、脉冲间隔、卡滞与发散检测、停稳门控、航向配平、平移纠偏、确定性重放、控制周期适应性、超时行为，以及刹车与滑行的区别。

## 接入项目

### 1. 实现输出回调

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

`BRAKE` 应使用平台能够提供的最强电气刹车。如果驱动无法主动刹车，应将其视为平台限制，而不是悄悄按 `COAST` 处理。

### 2. 初始化控制器

```c
skm_config_t config;
skm_t motion;

skm_config_default(&config);
/* 根据车辆调整增益和限幅。 */
if (skm_init(&motion, &config, motor_output, NULL) != SKM_API_OK) {
    /* 处理非法配置。 */
}
```

`skm_t` 由调用方分配，本库不使用堆。

### 3. 提供周期采样

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

在同一执行上下文中启动动作：

```c
if (skm_start_straight(&motion, 500.0f) == SKM_API_OK) {
    /* 请求已接受，下一次 tick 开始输出。 */
}
```

动作执行期间，启动接口返回 `SKM_API_BUSY`，当前动作不会被修改。轮速模式会持续运行，直到调用 `skm_abort()`。

## API 概览

| 函数 | 用途 |
|---|---|
| `skm_config_default()` | 填入默认配置 |
| `skm_init()` | 校验配置并初始化上下文 |
| `skm_start_wheel_speed()` | 以固定左右目标运行轮速内环 |
| `skm_start_straight()` | 保持航向移动带符号距离 |
| `skm_start_turn()` | 原地转动带符号角度 |
| `skm_abort()` | 请求在下一次 tick 刹车或滑行 |
| `skm_tick()` | 推进一个控制周期 |
| `skm_is_busy()` | 查询是否有动作正在执行 |
| `skm_get_status()` | 读取阶段、结果、剩余误差和目标 |
| `skm_get_learning()` | 读取学习到的刹车时间、配平和点动增益 |
| `skm_set_learning()` | 在空闲时恢复已保存的标定值 |
| `skm_reset_learning()` | 在空闲时恢复配置中的学习初值 |

动作结果：

| 结果 | 含义 |
|---|---|
| `SKM_RESULT_OK` | 最终误差在容差内 |
| `SKM_RESULT_ABORTED` | 由 `skm_abort()` 中止 |
| `SKM_RESULT_TIMEOUT` | 超过动作时间限制 |
| `SKM_RESULT_STALLED` | 运动进展不足 |
| `SKM_RESULT_OVERSHOOT` | 最终位置超出容差，不会倒车修正 |
| `SKM_RESULT_DIVERGED` | 反馈朝错误方向运动 |
| `SKM_RESULT_SENSOR_FAULT` | 采样无效、包含非有限值，或 `dt_s` 超出允许范围 |

## 控制行为

### 直行

里程环产生前进速度指令，并由 `accel_mms2` 限制变化率。航向环产生差速修正，轮速内环跟踪最终的左右目标。

当预测滑行距离接近剩余距离时开始刹车：

```text
滑行距离 = 前进速度 * 刹车安全系数 * 学习刹车时间
剩余距离 - 滑行距离 <= 0.5 * 位置容差时刹车
```

停稳后，控制器等待连续静止窗口，再测量实际滑行并更新刹车估计。只有有效且处于配置范围内的样本才会被接受。欠程时会再次向前逼近，不会为了修正小幅过冲而倒车。

### 航向配平

稳定巡航时，控制器按时间累计航向 PID 的纠偏量。直行动作成功结束后，将平均纠偏量的一部分写入前馈配平。输出饱和、低速、中止和失败动作都不会参与学习。

这可减小电机增益、轮组摩擦或传动损耗不一致造成的重复性偏差，但不能代替机械校正，也不能修复传感器符号接反。

### 原地转弯

大角度转弯先使用角度连续控制和静摩擦前馈。只有当前旋转正在减小角度误差时，才使用预测刹车。车辆停稳后，通过短开环脉冲逼近目标。

每次脉冲后都必须刹车并完成连续停稳确认。只有实际转角方向正确且位于允许范围内，才更新仿射脉冲模型。没有产生运动的脉冲会逐步增加下一次脉宽，但不会超过硬限幅；连续失败返回 `STALLED`。

小角度转弯跳过连续粗调，直接进入脉冲控制。

### 学习边界

学习值保存在 `skm_t` 中，不会自动写入持久存储。平台如果需要掉电保留，应在外部保存 `skm_get_learning()` 的结果，并通过 `skm_set_learning()` 恢复。中止、超时、传感器故障、反向运动和超出物理范围的样本都不会更新学习值。

## 主要配置项

| 字段 | 作用 |
|---|---|
| `speed_pi` | 轮速内环增益与归一化输出限幅 |
| `heading_pid` | 直行航向修正 |
| `turn_pid` | 转弯连续粗调 |
| `cruise_mms`、`accel_mms2` | 直行速度与指令斜坡 |
| `position_kp`、`position_tol_mm` | 里程响应与允许误差 |
| `heading_corr_max_mms` | 最大差速修正 |
| `heading_trim_beta` | 巡航平均纠偏写入配平的比例 |
| `turn_stiction_mms` | 接近目标时的最小有效转弯指令 |
| `turn_angle_tol_deg` | 原地转弯容差 |
| `straight_brake_*`、`turn_brake_*` | 刹车估计初值、范围和更新率 |
| `hop_*` | 脉冲时间、增益范围、输出幅度、重试和安全限制 |
| `*_timeout_s` | 动作总时长硬限制 |

默认值只是通用起点，建议按以下顺序调整：

1. 架空车轮，确认电机和 IMU 符号。
2. 在正常负载下整定 `speed_pi`。
3. 调整 `heading_pid`，在平整地面确认直行。
4. 在目标巡航速度下检查预测刹车和学习结果。
5. 调整转弯粗调，再调整脉冲幅度与限幅。
6. 用非法采样、架空车辆和堵转车轮验证故障行为。

## 仿真器

`skid_sim` 是一阶确定性模型，包含独立左右增益、静摩擦、不同的驱动/刹车/滑行响应和可选故障。

```c
skm_sim_config_t sim_config;
skm_sim_t sim;
skm_t motion;

skm_sim_config_default(&sim_config);
skm_sim_init(&sim, &sim_config);
skm_init(&motion, &config, skm_sim_output, &sim);
```

仿真器支持：

- 左右增益不一致
- 恒定共模扰动
- 车轮卡死
- 电机或陀螺符号反转
- 强制角速度，用于验证停稳门控
- 无效采样

它不模拟轮地滑移、悬架、轮胎形变、编码器量化和真实传感器噪声。仿真通过不代表实车性能已经验证。

## 限制与安全

- 控制器面向四轮滑移转向平台，不适用于麦轮或阿克曼底盘。
- 航向环要求连续 yaw 估计，并遵循本文档的符号约定。
- 输出是归一化值，适配层必须将其映射为安全的电机指令，并实施独立的电流或扭矩限制。
- `dt_s` 必须为正且不超过 1 秒，正常运行时建议使用固定控制周期。
- API 与 `skm_tick()` 不是线程安全的。
- `skm_abort()` 在下一拍生效，不等同于硬件急停。
- 电机和执行器供电应使用独立硬件急停，不应依赖软件时序保障人身安全。
- 落地测试前先架空车辆验证行为。

## 许可证

MIT，详见 [LICENSE](LICENSE)。
