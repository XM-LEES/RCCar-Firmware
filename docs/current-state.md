# 下位机代码结构与当前实现

本文细化根目录 `../../docs/系统架构与数据流.md`、`../../docs/接口与协议.md` 中和 STM32 下位机相关的部分。系统级阶段目标、ROS topic 和最终验收仍以根目录文档为准；本文件负责说明 `RCCar-new/` 里代码怎么组织、数据怎么流动、功能落在哪些文件里。

## 代码目录职责

| 路径 | 职责 | 修改边界 |
| --- | --- | --- |
| `Core/Src/main.c` | HAL 初始化、运行服务初始化、硬件版本检查、输入中断启动 | 外设启动顺序、TIM/UART/ADC 初始化入口变化时必须同步核对 `.ioc` 和 Keil 工程 |
| `Core/Src/freertos.c` | FreeRTOS 队列和任务创建 | 任务增删、优先级、栈大小变化必须同步本文和流程文档 |
| `Core/Src/usart.c` | `UART4` ROS 链路和 `USART1` TX debug 初始化 | UART 引脚、波特率、DMA/IRQ 变化必须同步根 `docs/接口与协议.md` |
| `Core/Src/tim.c` | `TIM4` RC 输入捕获、`TIM8` PWM 输出、`TIM6` runtime stats | 通道、周期、预分频变化必须同步本文 |
| `WHEELTEC_APP/SerialControl_task.c` | UART4 command 组帧、校验、flags 解析、RC 抢占时的串口命令门控 | 下行协议和安全入口变化必须同步根 `docs/接口与协议.md` |
| `WHEELTEC_APP/servo_basic_control.c` | Ackermann 执行、ESC/舵机映射、安全仲裁、RC 接管、通信超时、speed PI trim | 运动行为和安全边界变化必须同步根架构/协议和本文件 |
| `WHEELTEC_APP/servo_basic_output.c` | TIM8 CH1/CH2 PWM 输出绑定 | PWM 引脚或通道变化必须同步本文 |
| `WHEELTEC_APP/servo_rc_capture.c` | TIM4 CH1/CH2/CH3 RC pulse 捕获 | RC 通道、阈值来源变化必须同步本文 |
| `WHEELTEC_APP/hall_speed.c` | 霍尔计数、速度估算、方向应用 | 轮径、计数倍率、有效性规则变化必须同步根架构/协议 |
| `WHEELTEC_APP/data_task.c` | 24 字节 telemetry 打包、UART4 上行、USART1 debug mirror | 上行字段、状态位、长度变化必须同步根 `docs/接口与协议.md` |
| `WHEELTEC_APP/app_runtime_state.c` | 电压、debug level、UART DMA 统计和诊断状态 | 状态字段变化必须同步 telemetry 映射 |
| `WHEELTEC_APP/Inc/app_vehicle_config.h` | 车辆、ESC、舵机、RC、安全参数默认值 | 默认值变化必须说明来源和是否已实车复核 |
| `WHEELTEC_BSP/` | OLED、ADC、DWT、flash、buzzer 等板级支持 | 只保留当前 APP 调用或计划明确需要的 BSP |
| `MDK-ARM/WHEELTEC.uvprojx` | Keil 工程源码清单 | 增删源码必须跑工程路径检查 |
| `WHEELTEC.ioc` | CubeMX 外设配置源 | 外设变更必须和 `Core/Src/*`、Keil 工程一致 |

## 启动顺序

```text
main()
  -> HAL_Init()
  -> SystemClock_Config()
  -> MX_GPIO_Init()
  -> MX_DMA_Init()
  -> MX_USART1_UART_Init()
  -> MX_UART4_Init()
  -> MX_TIM4_Init()
  -> MX_TIM6_Init()
  -> MX_ADC1_Init()
  -> MX_TIM8_Init()
  -> ServoBasic_Init()
  -> DWT_Init()
  -> HallSpeed_Init()
  -> ADC_Userconfig_Init()
  -> HAL_TIM_Base_Start(TIM6)
  -> OLED init
  -> hardware version check
  -> HAL_TIM_IC_Start_IT(TIM4 CH1/CH2/CH3)
  -> HAL_UART_Receive_IT(UART4, 1 byte)
  -> MX_FREERTOS_Init()
  -> osKernelStart()
```

`MX_FREERTOS_Init()` 当前创建：

| 任务 | 文件 | 频率/触发 | 职责 |
| --- | --- | --- | --- |
| `ServoBasic_Task` | `servo_basic_control.c` | 约 50 Hz | 自动/RC 仲裁、PWM 输出、安全状态处理 |
| `SerialControlTask` | `SerialControl_task.c` | UART4 队列驱动 | 11 字节 command 解析并更新自动目标 |
| `RobotDataTransmitTask` | `data_task.c` | 20 Hz | 24 字节 telemetry 打包和发送 |
| `show_task` | `show_task.c` | 约 10 Hz | OLED 显示当前状态 |
| `StartInitTask` | `freertos.c` | 启动一次 | 蜂鸣器启动提示后退出 |

## 下行控制数据流

```text
UART4 RX IRQ
  -> HAL_UART_RxCpltCallback()
  -> g_xQueueROSserial
  -> SerialControlTask()
      - 搜索 0x7B 帧头
      - 固定 11 字节组帧
      - 校验 tail 和 BCC
      - 只接受 cmd=0x01
      - 解析 flags / speed_mmps / steering_mrad
      - RC 抢占时阻断非零自动运动命令
  -> ServoBasic_UpdateAckermannFromOrin()
  -> ServoBasic_Task()
      - enable/software-stop/timeout/RC 仲裁
      - speed 限幅和 ESC PWM 映射
      - steering 限幅和 servo PWM 映射
      - Hall speed PI trim
  -> ServoBasic_OutputEscPulse() / ServoBasic_OutputServoPulse()
  -> TIM8 CH1/CH2
```

下行固定为 11 字节，所有多字节数均为大端：

| 字节 | 含义 |
| --- | --- |
| 0 | 帧头 `0x7B` |
| 1 | 协议命令 `0x01`，表示直接 Ackermann 目标 |
| 2 | flags：bit0 `ENABLE`，bit1 `BRAKE`，bit2 `CLEAR_FAULT`，bit7 `SOFTWARE_STOP`；其余位必须为零 |
| 3..4 | `speed_mmps`，有符号 16 位，单位 `0.001 m/s` |
| 5..6 | `steering_mrad`，有符号 16 位，单位 `0.001 rad`；正值为左转 |
| 7..8 | 保留，必须为零 |
| 9 | 字节 0..8 的逐字节 XOR BCC |
| 10 | 帧尾 `0x7D` |

`SOFTWARE_STOP` 会立即把 ESC 和转向输出都置为配置中位；它不是物理急停。`BRAKE` 只是软件零速请求，不代表车辆有独立物理制动器。`CLEAR_FAULT` 只有在速度和转角字段都为零、没有停止覆盖、RC 没有接管且 RC emergency 未激活时才会被接受。串口任务只登记一次清除请求；20 Hz telemetry 任务在同一周期内清除 Hall/frame 历史诊断、重新计算当前 fault source，并且只在持续故障已经消失时清除 `FAULT_LATCHED`。

## 上行 telemetry 数据流

```text
Hall / servo estimate / battery / safety state
  -> RobotDataTransmitTask()
      - HallSpeed_GetState()
      - ServoBasic_GetAckermannFeedback()
      - update_power_state()
      - status_flags/status_bits 填充
      - 固定 24 字节 basebuffer
  -> UART4 DMA
  -> USART1 TX debug mirror when debug_level == 0
```

上行固定为 24 字节，所有多字节数均为大端：

| 字节 | 含义和来源 |
| --- | --- |
| 0 | 帧头 `0x7B` |
| 1 | 紧凑状态 flags，定义见下表 |
| 2 | 发送序号，8 位自然回绕 |
| 3..6 | `hall_delta_count`，有符号 32 位；计数由 Hall 直接测量，符号来自当前自动命令或 RC 油门方向；零命令后的滑行阶段保留最后方向 |
| 7..8 | `speed_mmps`，有符号 16 位；大小来自 Hall 周期，符号来自当前控制方向或滑行期间保留的最后方向；方向未知时为零且 valid 位清零 |
| 9..10 | `steering_mrad`，有符号 16 位；由实际输出 PWM 和舵机标定估算，不是转角传感器测量 |
| 11..12 | `yaw_rate_mradps`，有符号 16 位；由 Hall 速度、PWM 转角估算和轴距运动学计算，不是陀螺仪测量 |
| 13..14 | `battery_mv`，无符号 16 位，来自电池 ADC 换算 |
| 15..16 | `dt_ms`，无符号 16 位，上行帧间隔 |
| 17..20 | `status_bits`，无符号 32 位，定义见下表 |
| 21 | 固定协议标识 `0xA1` |
| 22 | 字节 0..21 的逐字节 XOR BCC |
| 23 | 帧尾 `0x7D` |

byte 1 flags：bit0 `AUTO_ENABLED`，bit1 `RC_OVERRIDE_ACTIVE`，bit2 `STOP_OVERRIDE_ACTIVE`，bit3 `COMMAND_TIMEOUT`，bit4 `BRAKE_ACTIVE`，bit5 `FAULT_LATCHED`，bit6 `STEERING_IS_MEASURED`，bit7 保留。当前没有转角传感器，因此 bit6 始终为零。

`status_bits`：bit0 `FAULT_LATCHED`，bit1 `COMMAND_TIMEOUT`，bit2 `RC_OVERRIDE_ACTIVE`，bit3 `STOP_OVERRIDE_ACTIVE`，bit4 `BRAKE_ACTIVE`，bit5 `AUTO_ENABLED`，bit6 `HALL_FEEDBACK_VALID`，bit7 `HALL_FAULT`，bit8 `STEERING_ESTIMATE_VALID`，bit9 `STEERING_IS_MEASURED`，bit10 `RC_INPUT_FAULT`，bit11 `BATTERY_VALID`，bit12 `HALL_STANDSTILL_CONFIRMED`，bit14 `SPEED_SATURATED`，bit15 `STEERING_SATURATED`，bit16 `ACCEL_LIMITED`，bit17 `STEERING_RATE_LIMITED`，bit18 `FRAME_ERROR_SEEN`；bit13 和 bit19..31 保留。bit6 表示当前速度幅值和控制派生方向均可用；bit12 表示当前控制方向为零，并且从最后一个有效 Hall 脉冲和最近一次进入零方向请求两者中较晚的时刻起，已经经过自适应无脉冲窗口。bit6 和 bit12 互斥。bit12 是 Hall 可观测范围内的静止推定，不是独立物理停止传感器。bit7 表示上次被接受的 `CLEAR_FAULT` 请求后累计过被拒绝的过短 Hall 脉冲，是可清除的历史诊断，不是当前瞬时 Hall 失效。bit10 只表示 throttle/steering 捕获异常持续超过 `APP_RC_GLITCH_FREEZE_MS=100 ms`，或已启用 guard 后 guard 输入异常；默认关闭的 guard 不参与该位。bit18 是上次被接受的 `CLEAR_FAULT` 请求后的 UART4 错帧历史诊断。`ACCEL_LIMITED` 仅表示固件内部速度目标变化斜坡生效，不表示车辆实际加速度受控或被测量。电池仅按原始 `battery_mv` 上报，不在固件内设置低压阈值、故障或控制动作。

任何向 UART4 混发 ASCII、调试帧或其他长度数据的改动都会破坏固定长度解析；当前 UART4 只允许上述两个二进制帧。

## 当前实现与阶段目标差异

本节只说明当前代码事实，不替代 Windows/Keil 编译、烧录和真实车辆验证。

当前已经实现并可作为代码事实核对的内容：

- UART4 11 字节 Ackermann command 解析：帧头、帧尾、BCC、`cmd=0x01`、`ENABLE`、`BRAKE`、`CLEAR_FAULT`、`SOFTWARE_STOP`。其中停止相关 bit 只代表软件请求，不是物理急停或物理制动。
- 正式自动控制入口使用 `speed_mmps + steering_mrad`；旧 `vx/vy/wz` 作为历史协议事实保留。
- RC 接管、软件停止请求和通信超时回中仲裁；未经确认的 RC guard 默认关闭。
- 速度和转角执行端限幅、速度目标变化斜坡、转角速度限幅、ESC/舵机 PWM 映射、霍尔速度反馈和 speed PI trim。速度目标斜坡不是实际加速度控制。
- UART4 24 字节 telemetry：`hall_delta_count`、`speed_mmps`、`steering_mrad`、`yaw_rate_mradps`、`battery_mv`、`dt_ms`、`status_bits`，byte 21 固定为协议标识 `0xA1`。
- 当前填充的状态：`FAULT_LATCHED`、`AUTO_ENABLED`、`RC_OVERRIDE_ACTIVE`、`STOP_OVERRIDE_ACTIVE`、`COMMAND_TIMEOUT`、`BRAKE_ACTIVE`、`HALL_FEEDBACK_VALID`、`HALL_STANDSTILL_CONFIRMED`、`HALL_FAULT`、`STEERING_ESTIMATE_VALID`、`RC_INPUT_FAULT`、`BATTERY_VALID`、`SPEED_SATURATED`、`STEERING_SATURATED`、`ACCEL_LIMITED`、`STEERING_RATE_LIMITED`、`FRAME_ERROR_SEEN`。
- `CLEAR_FAULT` 当前通过跨任务请求计数交给 telemetry 任务结算：清除 Hall fault count 和 UART4 frame-error 历史诊断，随后重新计算当前 fault source；持续 RC 输入故障仍在时不会清除 `FAULT_LATCHED`。

当前尚未实现但仍属于协议目标/阶段验收要求的内容：

- 真实转角测量硬件接入后的 `STEERING_IS_MEASURED`；当前转角是舵机标定估算，不得把该位声明为实测。
- `FAULT_LATCHED`、`RC_INPUT_FAULT`、`SPEED_SATURATED`、`STEERING_SATURATED`、`ACCEL_LIMITED` 和 `STEERING_RATE_LIMITED` 已有代码路径和静态契约检查；仍需要 Windows/Keil 编译、烧录和实车/台架测试记录确认。

修改任何字段、位定义或单位时，必须同步修改上位机协议编解码和永久测试。

当前已落地的固件静态契约检查：

```bash
python3 tools/acceptance/check_firmware_contract.py --workspace-root .
```

该检查覆盖 UART4 Ackermann command 帧长、cmd/flags、BCC、RC override guard、24 字节 telemetry、当前已实现状态位赋值、UART4 `115200 8N1`。阶段 1 目标状态位检查使用：

```bash
python3 tools/acceptance/check_firmware_contract.py --workspace-root . --require-phase1-status-bits
```

PC 串口测试使用同一 UART4 Ackermann 正式协议。固件不增加 PC-only 自动入口；测试工具负责串口打开、正式帧收发、raw log 和 parsed telemetry 摘要。

## RC 接管数据流

```text
TIM4 CH1/CH2/CH3 input capture
  -> HAL_TIM_IC_CaptureCallback()
  -> ServoRC_IC_CaptureCallback()
  -> ServoBasic_Task()
      - throttle / steering 偏离中位触发 RC passthrough
      - throttle 相对中位给霍尔 delta 提供前进/后退方向符号
      - RC 接管期间阻断非零串口目标，并用零速/中位目标刷新串口新鲜度，避免 `250 ms` 命令超时阻塞 `500 ms` 回中释放保持
      - 区分“无串口时的空闲 RC 直通”和“摇杆真实接管”：前者在串口恢复且摇杆居中时立即释放，后者必须完成 `500 ms` 回中保持
      - 单个非法捕获先冻结在最后一次有效值；只有连续超过 `100 ms` 才上报 `RC_INPUT_FAULT`，默认关闭的 guard 捕获不会触发该故障
      - 回中并保持释放时间后恢复 automatic
  -> TIM8 CH1/CH2 PWM
```

当前通道约定：

- `TIM4_CH1`：RC throttle。
- `TIM4_CH2`：RC steering。
- `TIM4_CH3`：历史 RC guard 捕获通道；当前默认关闭，不能描述成物理急停。
- `TIM8_CH1`：ESC PWM。
- `TIM8_CH2`：steering servo PWM。

## 霍尔速度反馈路径

```text
PE13/PE14 Hall GPIO
  -> HAL_GPIO_EXTI_Callback()
  -> HallSpeed_OnCountEvent()
  -> HallSpeed_GetState()
  -> telemetry hall_delta_count / speed_mmps
  -> ServoBasic_GetAckermannFeedback()
  -> speed PI trim
```

当前代码事实：

- 霍尔速度换算常量在 `WHEELTEC_APP/hall_speed.c`。
- `HALL_WHEEL_DIAMETER_M = 0.230 m` 和 `HALL_COUNT_EVENTS_PER_REV = 10` 是当前代码事实。
- 当前采用 `counts_per_meter = 10 / (pi * 0.230) = 13.840`。
- 霍尔计数硬件不区分前进/后退方向；速度和 `hall_delta_count` 的符号来自当前执行控制方向。
- 自动 Ackermann 模式下，方向来自上位机下发的 `speed_mps` 正负号。
- RC passthrough/manual 模式下，方向来自 RC throttle PWM 相对 `1500 us` 中位和 `APP_RC_THROTTLE_NEUTRAL_HOLD_US` 死区的正负偏移。
- 非零命令结束后，测速方向保留到 Hall 无脉冲窗口确认静止，因此滑行阶段仍可上报带最近控制方向符号的速度和 `hall_delta_count`。确认静止后方向清零。
- 启动后无脉冲至少 `0.5 s`；或进入零方向请求后，并且最后一个脉冲后，两段时间都超过 `clamp(2 * last_period, 0.5 s, 4.0 s)` 时，bit12 `HALL_STANDSTILL_CONFIRMED` 才置位。非零运动命令期间没有 Hall 脉冲属于未知反馈，不得置位；停止请求必须重新完成静默确认窗口。霍尔周期和静默窗口保存原始 DWT cycle 时间戳，先以无符号差值处理 32 位计数器回卷，再换算为微秒；超时后的第一个脉冲只重建周期原点，不能恢复陈旧速度。
- throttle 中位、RC throttle 不可用、自动命令为零且没有可保留的最近方向，或外力推动车辆时方向未知，telemetry 不能提供带符号位移；这类数据不能作为独立方向测量证据。
- 方向符号仍是命令方向代理，不是独立物理方向测量。倒车、滑行、刹车后惯性移动或外力推动车时，不能当作独立方向测量。

当前车辆标定默认值：

- 轴距 `600 mm`，来源为前轮中心到后轮中心实测距离。
- 轮距 `500 mm`，指同轴左右车轮中心平面间距；车辆最大外廓宽度为 `600 mm`。
- 车轮直径 `0.230 m`，对应半径 `115 mm`。
- 最大前轮转角左右均 `15°`，固件默认 `APP_ORIN_ACKERMANN_MAX_STEERING_MRAD=262`。
- 舵机中位暂用 `1500 us`，两侧端点 `1105 us` / `1895 us`，固件默认 range `395 us`。
- 自动控制速度命令限幅：前进和倒车均为 `3000 mm/s`，即 `3.00 m/s`；不存在旧 `0.5 m/s` 产品限速回退值。
- 自动控制速度命令满足 `abs(speed_mmps) < 300` 时，速度目标回零且 ESC 输出中位；转向目标仍按命令执行。恰好 `300 mm/s` 可以进入速度映射。该阈值不用于篡改有效的 Hall 实际速度反馈。
- 霍尔反馈超速保护阈值：`APP_HALL_SPEED_LIMIT_MMPS=5000`，释放阈值 `APP_HALL_SPEED_LIMIT_RELEASE_MMPS=4500`。这是异常保护，不是自动控制命令限速。
- 左右转方向沿用既有 Ackermann 协议：正转角为左转。

## 配置和调试入口

默认配置集中在 `WHEELTEC_APP/Inc/app_vehicle_config.h`。常用 Keil Watch 变量：

- `g_state.control_mode`
- `g_state.esc_pulse_us`
- `g_state.servo_pulse_us`
- `g_orin_state.active`
- `g_orin_state.auto_enabled`
- `g_orin_state.brake_active`
- `g_orin_state.feedback_vx_mps`
- `g_orin_state.feedback_vz_rad_s`
- `g_orin_state.feedback_steering_angle_rad`
- `g_hall_speed_state.event_count_total`
- `g_hall_speed_state.last_period_us`
- `g_hall_speed_state.direction`
- `g_hall_speed_state.command_direction`
- `g_hall_speed_state.stationary_confirmed`
- `g_hall_speed_state.fault_count`
- `g_speed_pi_enable`
- `g_speed_pi_feedback_valid`
- `g_speed_pi_error_mps`
- `g_speed_pi_trim_us`
- `g_speed_pi_final_us`
- `g_app_runtime_state.voltage_v`
- `g_app_runtime_state.debug_level`
- `g_app_runtime_state.uart4_rx_frame_error_seen`
- `g_app_runtime_state.fault_clear_request_count`
- `g_app_runtime_state.uart4_tx_busy_count`
- `g_app_runtime_state.uart4_tx_error_count`
- `TIM8->CCR1`
- `TIM8->CCR2`

## 修改边界

- 改 UART4 command 或 telemetry：同时核对 `SerialControl_task.c`、`data_task.c`、根 `docs/接口与协议.md`、上位机串口桥。
- 改安全行为：同时核对 `servo_basic_control.c`、`data_task.c` 状态位、根 `docs/系统架构与数据流.md`。
- 改霍尔/速度反馈：同时核对 `hall_speed.c`、`servo_basic_control.c`、`data_task.c`、根 odom/telemetry 文档。
- 改 RC 输入或 PWM 输出：同时核对 `tim.c`、`servo_rc_capture.c`、`servo_basic_output.c`、`WHEELTEC.ioc`、Keil 工程。
- 改 Keil 工程或 `.ioc`：必须跑工程源码路径检查，并确认 `Core/Src/*` 生成代码没有重新引入旧外设入口。

## 非当前主路径

以下内容不属于当前主路径，不能只因为厂家资料里存在就恢复：

- 旧 `vx/vy/wz` 串口控制。
- 旧 CAN 舵机/编码器链路。
- USART3/RS485。
- Bluetooth/App、USB HID gamepad、AutoRecharge、Ranger、ICM20948/IMU、RGB APP。

如需恢复上述能力，先在根目录阶段计划和接口文档中定义需求，再从 git 历史或厂家资料恢复对应实现。
