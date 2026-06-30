# AGX Orin -> STM32 串口运动学与 Ackermann PWM 链路说明

## 概述
当前工程支持 AGX Orin 通过 UART 向 STM32 发送 11 字节速度帧。STM32 在 `SerialControl_task.c` 中解析 `Vx/Vy/Vz` 后，当前正式运动控制路径只进入 Ackermann PWM 链路：

- `ServoBasic_UpdateFromOrin()`
- `servo_basic_control.c`
- `TIM8 CH1/CH2 -> PC6/PC7`

旧的 WHEELTEC 通用底盘控制队列和多车型 `RobotControl_task` 兼容层已从当前 APP 中删除。

## 几何参数来源
当前 Ackermann 几何以 `WHEELTEC_APP/Inc/app_vehicle_config.h` 为准：

- `wheelbase = 0.600 m`
- `track_width = 0.470 m`
- `wheel_radius = 0.115 m`
- `wheel_diameter = 0.230 m`
- `max_steering_angle = 0.262 rad`
- `base_link = rear axle center`

说明：
- `track_width` 与 `wheel_radius` 当前作为车辆参数和标定依据保留；
- 单舵机版本的 v1 Ackermann PWM 只直接使用 `wheelbase` 与 `max_steering_angle`。

## 串口协议
控制帧固定为 11 字节：

```text
0x7B  CMD1  CMD2  XH  XL  YH  YL  ZH  ZL  BCC  0x7D
```

字段含义：
- `CMD1 = 0x00`：正常速度控制
- `CMD2 bit7`：`flag_stop`
- `X/Y`：单位 `mm/s`
- `Z`：单位 `0.001 rad/s`
- `BCC`：前 9 字节 XOR

示例：
- `Vx = 0.12 m/s, Vz = 0`

```text
7B 00 00 00 78 00 00 00 00 03 7D
```

## 串口到 PWM 的数据路径
1. `HAL_UART_RxCpltCallback()` 将字节写入 `g_xQueueROSserial`
2. `SerialControlTask()` 组帧、做 BCC 校验、解析 `Vx/Vy/Vz`
3. 若 RC 未抢占：
   - 调用 `ServoBasic_UpdateFromOrin(vx, vy, vz, flag_stop)`
4. `servo_basic_control.c` 将速度命令映射为：
   - `ESC PWM -> TIM8_CH1 -> PC6`
   - `Servo PWM -> TIM8_CH2 -> PC7`

补充：
- 当 `RC override == 1` 时，`SerialControl_task.c` 会阻断非零自动驾驶命令；
- 刚进入 RC 抢占时，会自动发送一次零速命令，清除旧自动驾驶目标。

## Ackermann 转向映射
当前工程已实现 Ackermann 自行车模型，而不是旧的 `Vz -> 舵机 PWM` 线性映射。

公式：

```text
delta = atan(L * wz / vx)
```

其中：
- `L = 0.600 m`
- `wz = Vz`
- `vx = Vx`
- `delta` 为等效前轮转角

约束：
- `|delta| <= 0.262 rad`
- `delta = 0` 映射到 `g_orin_servo_center_us`
- `±0.262 rad` 映射到 `g_orin_servo_center_us ± g_orin_servo_range_us`

低速保护：
- 当 `|vx| < 0.05 m/s` 时：
  - ESC 输出回中
  - Servo 输出回中
  - 不继续根据 `wz / vx` 放大转向

倒车行为：
- 由于公式直接使用带符号 `vx`，倒车时相同 `wz` 会得到相反符号的舵机命令。

## 纵向速度到 ESC 的映射
当前 `Vx -> ESC PWM` 使用分段标定模型，不再使用单一线性满量程映射。

规则：
- `|vx| <= deadband`：输出 `ESC center`
- `vx > deadband`：映射到 `[forward_start_us, forward_max_us]`
- `vx < -deadband`：映射到 `[reverse_start_us, reverse_max_us]`

默认参数：
- `g_orin_vx_scale = 1000`
- `g_orin_vx_forward_cap_mmps = 3000`
- `g_orin_vx_reverse_cap_mmps = 3000`
- `g_orin_vx_deadband_mmps = 50`
- `g_orin_esc_forward_start_us = 1560`
- `g_orin_esc_reverse_start_us = 1440`
- `g_orin_esc_forward_max_us = 1650`
- `g_orin_esc_reverse_max_us = 1350`

设计目的：
- 避免架起测试时轮速过快
- 预留实车速度标定入口
- 允许前进和后退分别标定

## RC 抢占逻辑
当前 RC 策略不是“模式通道切换”，而是“默认 autonomous，RC 人为介入时自动抢占”。

输入定义：
- `PD12`：throttle
- `PD13`：steering
- `PD14`：guard / emergency

默认阈值：
- `g_rc_override_center_us = 1500`
- `g_rc_override_enter_threshold_us = 60`
- `g_rc_override_exit_threshold_us = 40`
- `g_rc_override_enter_samples = 2`
- `g_rc_override_release_hold_ms = 500`

行为：
- 无串口 PWM 控制流时，RC 直接直通，不需要先越过抢占阈值
- 有串口 PWM 控制流时，油门或转向偏离中位超过 `60 us`，并连续满足 2 个周期，则进入 RC 抢占
- 回到中位 `±40 us` 内并持续 `500 ms`，则恢复自动驾驶
- `PD14` 触发时立即进入保障/急停覆盖

## 可调参数
默认值集中在 `WHEELTEC_APP/Inc/app_vehicle_config.h`；以下 `g_orin_*`
变量仍可通过 Keil Watch 临时覆盖。

Ackermann 几何：
- `g_orin_ackermann_wheelbase_mm`
- `g_orin_ackermann_track_width_mm`
- `g_orin_ackermann_wheel_radius_mm`
- `g_orin_ackermann_max_steering_millirad`
- `g_orin_ackermann_min_vx_mmps`

ESC 标定与限速：
- `g_orin_vx_scale`
- `g_orin_vx_forward_cap_mmps`
- `g_orin_vx_reverse_cap_mmps`
- `g_orin_vx_deadband_mmps`
- `g_orin_esc_forward_start_us`
- `g_orin_esc_reverse_start_us`
- `g_orin_esc_forward_max_us`
- `g_orin_esc_reverse_max_us`

Servo：
- `g_orin_servo_center_us`
- `g_orin_servo_range_us`

RC 抢占：
- `g_rc_override_enter_threshold_us`
- `g_rc_override_exit_threshold_us`
- `g_rc_override_enter_samples`
- `g_rc_override_release_hold_ms`

## 推荐调试顺序
1. 先确认 `g_state.control_mode = SERVO_CTRL_MODE_AUTONOMOUS`
2. 发送固定低速串口帧，观察 `TIM8->CCR1 / CCR2`
3. 再发送不同 `Vz` 的帧，验证转向是否按 Ackermann 变化
4. 若实车速度偏差较大，先调：
   - `g_orin_esc_forward_start_us`
   - `g_orin_esc_forward_max_us`
   - `g_orin_vx_scale`
5. 若低速 RC 无法抢占，再调：
   - `g_rc_override_enter_threshold_us`
   - `g_rc_override_enter_samples`

## 当前仍未覆盖的内容
- 当前链路仍未实现真正的闭环车速控制；
- 纵向速度匹配依赖 ESC 实测标定，而不是仅靠 `wheel_radius` 自动换算；
- `Vy` 目前不参与 PWM 映射；
- 若后续需要左右独立前轮转角或完整 Ackermann 左右轮几何，需新增双舵机或转角分配层。
