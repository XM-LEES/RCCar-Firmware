# WHEELTEC Ackermann PWM 控制说明

## 概述
当前工程已经收敛为单一 Ackermann PWM 控制平台，正式控制入口只有两条：

- `ROS 串口 -> Ackermann PWM`
- `RC -> Ackermann PWM / 抢占`

旧差速/多车型底盘控制、旧轮速反馈、旧 CAN 舵机协议已从正式控制主路径中移除，不再参与车辆实际运动控制与 ROS 速度回传。

补充资料、厂家手册和历史排障记录已整理到 `docs/`。

## 当前正式控制链路
### 1. ROS 串口控制
- 入口文件：`WHEELTEC_APP/SerialControl_task.c`
- 输入协议：固定 11 字节速度帧
- 处理流程：
  - `HAL_UART_RxCpltCallback()` -> `g_xQueueROSserial`
  - `SerialControlTask()` 解析 `Vx/Vy/Vz`
  - `ServoBasic_UpdateFromOrin()` 映射为 ESC / Servo PWM

### 2. RC 接管控制
- 输入捕获：`TIM4`
- 引脚：
  - `PD12 -> TIM4_CH1`：RC throttle
  - `PD13 -> TIM4_CH2`：RC steering
  - `PD14 -> TIM4_CH3`：RC guard
- RC 仅在明显偏离中位时抢占自动驾驶，回中保持一段时间后自动恢复 autonomous。

### 3. PWM 输出
- 输出定时器：`TIM8`
- 引脚：
  - `PC6 -> TIM8_CH1`：ESC PWM
  - `PC7 -> TIM8_CH2`：Servo PWM
- 只初始化和启动 `TIM8_CH1/CH2`；`PC8/PC9` 当前不作为控制输出。

### 4. 霍尔轮速反馈（阶段一）
- 当前阶段新增双路霍尔轮速输入，仅用于替换 `ROS` 上行 `vx` 伪反馈，不参与 `ESC` 闭环控制。
- 接线规划：
  - `PE13`：`Hall A`
  - `PE14`：`Hall B`
- 计数方式固定为：
  - 保留当前 `A/B` 角色互换方案：`Hall B` 作为计数脚
  - `Hall A/B` 使用双边沿中断观察 `01 -> 00 -> 10` / `10 -> 00 -> 01` 方向序列
  - 仅在 `Hall B` 的下降沿记录一次有效事件并更新测速周期
  - 当前默认 `HALL_COUNT_EVENTS_PER_REV = 10`，对应新轮子 `10` 个触发磁铁
- 当前轮径按 `0.235 m` 计算，轮周长约 `0.738 m`
- 当前阶段不启用 `PID`，只做真实测速与反馈替换

#### 霍尔输入电气前提
- 双路霍尔信号均按 `3.3V` 上拉、低电平有效处理
- 霍尔传感器与主控必须共地
- `PE13/PE14` 输入电平不得超过 `3.3V`
- 若实测高电平可能高于 `3.3V`，必须先做电平转换或隔离
- `PA13/PA14` 为 `SWD` 下载调试口，不能占用
- `PC6/PC7` 已为正式 `ESC/Servo PWM` 输出，不能改作轮速输入

## Ackermann 几何参数
当前几何参数以 `EXTRINSICS.md` 为准：
- `wheelbase = 0.54 m`
- `track_width = 0.48 m`
- `wheel_radius = 0.11 m`
- `max_steering_angle = 0.393 rad`
- `base_link = rear axle center`

默认值集中在 `WHEELTEC_APP/Inc/app_vehicle_config.h`；代码中对应的
Keil Watch 运行时参数为：
- `g_orin_ackermann_wheelbase_mm`
- `g_orin_ackermann_track_width_mm`
- `g_orin_ackermann_wheel_radius_mm`
- `g_orin_ackermann_max_steering_millirad`


## 串口协议

当前 ROS 串口协议以 `docs/protocols/serial-protocol.md` 的“当前固件 ROS UART 协议”为准；厂家表格仅作为历史资料对照。

### ROS -> STM32 下行控制帧
固定 11 字节：

```text
0x7B  CMD1  CMD2  XH  XL  YH  YL  ZH  ZL  BCC  0x7D
```

含义：
- `CMD1 = 0x00`：速度控制
- `CMD2 bit7`：`flag_stop`
- `X/Y`：单位 `0.001 m/s`
- `Z`：单位 `0.001 rad/s`
- `BCC`：前 9 字节异或

### STM32 -> ROS 上行反馈帧
固定 24 字节基础帧：

```text
0x7B flag vx_h vx_l vy_h vy_l wz_h wz_l imu0 ... imu11 bat_h bat_l bcc 0x7D
```

说明：
- 字段真实/虚拟状态以 `docs/protocols/serial-protocol.md` 为准
- `vx` 是真实霍尔轮速反馈；无有效霍尔测速时填 `0`
- `vy` 当前固定为 `0`，只是保留阿克曼底盘不使用的横向速度槽位
- `wz` 由霍尔 `vx`、当前舵机 PWM 和轴距估算，不是 IMU 实测
- `g_orin_feedback_scale` 只保留给旧的 PWM 估算反馈，不再作用于霍尔真轮速
- 默认值为 `1254`（按千分比），即 `1.254x`
- 原 IMU 字段 `8..19` 保留为协议占位，当前固件固定填 `0`
- `battery` 单位为 mV
- 常规模式下 `UART4 -> ROS` 发送这 24 字节基础帧
- 不向 ROS 这一路混发霍尔调试帧、`19` 字节超声波帧或 `8` 字节回充帧
- 霍尔调试信息通过 OLED 和 Keil Watch 观察

## 核心接口
文件：`WHEELTEC_APP/Inc/servo_basic_control.h`

保留的正式接口：
- `ServoBasic_Init()`
- `ServoBasic_ProcessControl()`
- `ServoBasic_UpdateFromOrin(...)`
- `ServoBasic_GetState()`
- `ServoBasic_IsRcOverrideActive()`
- `ServoBasic_IsRcEmergencyActive()`
- `ServoBasic_GetOrinFeedback(...)`
- `ServoBasic_OutputEscPulse()`
- `ServoBasic_OutputServoPulse()`

已移出正式主路径：
- 旧差速/多车型底盘控制队列
- 旧 CAN 舵机协议入口
- 旧轮速反馈驱动 ROS 速度回传的逻辑
- 旧 RC joystick 输入
- RGB 灯条运行时任务
- Bluetooth/App 控制
- USB HID 手柄控制
- 自动回充 `AutoRecharge`
- 超声波 `Ranger`

## RC 抢占与安全策略
- 默认控制模式：`AUTONOMOUS`
- RC 偏离中位超过阈值后自动进入 `RC_PASSTHROUGH`
- RC 回中并保持释放时间后恢复 `AUTONOMOUS`
- `PD14 guard` 可触发紧急覆盖
- 无有效自主目标且 RC 不可用时，输出退回：
  - `ESC neutral`
  - `Servo center`

## 常用调试变量
建议在 Keil Watch 中观察：
- `g_state.control_mode`
- `g_state.esc_pulse_us`
- `g_state.servo_pulse_us`
- `g_hall_speed_state.event_count_total`
- `g_hall_speed_state.last_period_us`
- `g_hall_speed_state.direction`
- `g_hall_speed_state.fault_count`
- `g_rc_override_active`
- `g_rc_guard_active`
- `g_orin_state.active`
- `g_orin_state.feedback_vx_mps`
- `g_orin_state.feedback_vz_rad_s`
- `g_orin_feedback_scale`
- `g_app_runtime_state.voltage_v`
- `g_app_runtime_state.debug_level`
- `g_app_runtime_state.uart4_tx_busy_count`
- `g_app_runtime_state.uart4_tx_error_count`
- `g_app_runtime_state.usart1_debug_tx_busy_count`
- `g_app_runtime_state.usart1_debug_tx_error_count`
- `TIM8->CCR1`
- `TIM8->CCR2`

## 当前保留但不属于正式控制主路径
以下内容仍可作为调试或附属功能保留，但不再决定车辆正式控制行为：
- `USART1 TX` 调试输出；`USART1 RX` 不再启动接收
- 板载 OLED 显示
- `bsp_flash.c` 仍在 Keil 目标内保留，用作未来参数断电保存能力；当前 APP 不再读取旧默认速度/纠偏参数

## 已清理的旧功能
以下旧功能已从源码和 Keil 工程编译项中移除：
- Bluetooth/App 控制：删除 UART2 App 接收队列、蓝牙任务和 App 显示发送任务
- USB HID 手柄控制：删除 `USB_HOST/` 和 `bsp_gamepad.*`，移除 HCD/USB Host 编译入口
- 自动回充 `AutoRecharge`：删除回充任务、回充命令处理和回充附加上报
- 超声波 `Ranger`：删除 Ranger 采集模块、避障参数和 Ranger 附加上报
- ICM20948/IMU：删除 ICM 驱动、寄存器定义和 IMU 任务；ROS 上行 24 字节帧保留 IMU 占位并固定填 `0`
- RGB APP 运行时：删除 `RGBStripControl_task.*`，ROS `cmd1=4` RGB 帧保持兼容但当前忽略
- 旧 RC joystick 和旧 `RobotControl_task` 兼容层：删除旧输入采集、旧控制队列和旧任务；reset/log 串口 helper 已内联到 `SerialControl_task.c`
- 旧 CAN 舵机驱动：`bsp_ServoDrive.*` 已在当前清理版本删除，未来需要时从 git 历史或厂家参考代码恢复
- 旧车型选择/运行参数层：删除 `robot_select_init.*`、`Robot_Select()`、`RobotHardWareParam` 和 `RobotControlParam`；当前只保留 `app_runtime_state.*` 里的电压、debug level 和 UART DMA 统计
- CAN 当前工程入口：`MX_CAN1_Init()` / `MX_CAN2_Init()`、CAN IRQ、HAL CAN 编译开关、Keil `can.c` / `stm32f4xx_hal_can.c` 编译项和 `WHEELTEC.ioc` CAN 配置已移除；当前清理版本同时删除 `Core/Src/can.c` / `Core/Inc/can.h`，未来需要 CAN 时从 git 历史或厂家参考代码恢复
- USART3/RS485 当前入口：移除 `MX_USART3_UART_Init()`、USART3 RX DMA、USART3 IRQ、PB10/PB11 USART3 引脚和 `WHEELTEC.ioc` USART3 配置；当前上位机链路只走 `UART4`
- 当前产品不用的 BSP 历史源码：删除 `bsp_RGBLight.*`、`bsp_siic.*`、`bsp_eeprom.*`、`bsp_key.*`、`bsp_RTOSdebug.*`、`bsp_led.*`，未来需要时从 git 历史或厂家参考代码恢复
- 当前产品不用的 Core/IOC 硬件入口：移除 TIM9/TIM11 RGB PWM、TIM4_CH4/PD15、PB6/PB7 software IIC、UserKey、UserLED 和 ENKey 配置；保留 VersionBit 硬件版本检测
- Hall 32 字节调试帧和 IRQ/Callback/有效边沿调试计数：删除串口混发风险，仅保留 `g_hall_speed_state`
- `WHEELTEC.ioc` 已同步移除 USB Host、USB OTG FS、Bluetooth/App USART2、USART2 TX DMA、Ranger TIM2/TIM3、超声波 GPIO、CAN1/CAN2、CAN NVIC、CAN 引脚、USART3/RS485、USART1 RX、TIM9/TIM11、TIM4_CH4/PD15、RGB/IIC/按键/用户 LED/ENKey

保留路径不变：
- `UART4` ROS 下行控制与 24 字节上行基础帧
- `TIM4 CH1/CH2/CH3` RC 接收器接管；不再配置 `TIM4_CH4`
- `PE13/PE14` 霍尔轮速反馈
