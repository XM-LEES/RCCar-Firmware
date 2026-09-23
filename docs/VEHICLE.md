# 车辆数据

数值以当前工作树源码为准。下表记录代码配置值及其单位，不将配套说明中的不同数值覆盖源码；空白项不表示零值。

主要来源：[车辆配置](../WHEELTEC_APP/Inc/app_vehicle_config.h)、[Hall 参数](../WHEELTEC_APP/hall_speed.c)、[控制实现](../WHEELTEC_APP/servo_basic_control.c)、[AUTO门控](../WHEELTEC_APP/mode2_drive_gate.c)、[纵向PID](../WHEELTEC_APP/longitudinal_controller.c)。源码中的其余有效配置同样沿用，不因未列入本表而取消。

## 几何

速度正向为前进，前轮角正向为左转。

| 数据 | 数值 | 源码依据 |
| --- | --- | --- |
| 轴距 | 0.600 m | `APP_ORIN_ACKERMANN_WHEELBASE_MM=600` |
| 轮距 | 0.500 m | `APP_ORIN_ACKERMANN_TRACK_WIDTH_MM=500` |
| 轮半径 | 0.115 m | `APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM=115` |
| 轮径 | 0.230 m | `HALL_WHEEL_DIAMETER_M=0.230f` |
| 计算周长 C | 0.722566 m | `HALL_WHEEL_CIRCUMFERENCE_M=π×轮径`；非实测滚动周长 |
| 等效前轮角上限 | ±0.349 rad | `APP_ORIN_ACKERMANN_MAX_STEERING_MRAD=349` |
| 轮宽（m） | | |
| 车体包络长／宽／高（m） | | |
| 实测滚动周长（m） | | |

## 输出与控制参数

| 参数 | 代码配置值 | 含义 |
| --- | --- | --- |
| PWM 周期 / tick | 2631 / 1 µs | TIM8：PSC=167、ARR=2630 |
| ESC 脉宽输入范围 / 中点 | 1000–2000 / 1500 µs | AUTO物理端点与中点 |
| 转向中点 / 幅度 | 1500 / ±395 µs | 有效范围 1105–1895 µs |
| 转向角—脉宽 | −0.349 / 0 / +0.349 rad ↔ 1105 / 1500 / 1895 µs | 正左转对应 PWM 增大 |
| 速度目标处理 | 无缩放、无软件死区、无速度上限 | 精确零表示停车；非零目标直接进入PID与模式二门控 |
| 转角变化率 | 0.900 rad/s | `APP_ORIN_STEERING_RATE_LIMIT_MRADPS=900` |
| PID Kp / Ki / Kd | 120.0 / 20.0 / 0.0 | Kp：µs/(m/s)；Ki：µs/[(m/s)·s]；Kd：µs/(m/s²)；工程初值，需实车标定 |
| PID微分滤波 / 积分跟踪时间 | 80 / 100 ms | `APP_SPEED_PID_DERIVATIVE_TAU_MS` / `APP_SPEED_PID_TRACKING_TAU_MS` |
| AUTO最小刹车偏移 | 50 µs | `APP_AUTO_BRAKE_MIN_US`；需实车标定，不代表已测制动力 |
| 前进刹车进入误差 | max(0.20 m/s, 0.10×目标) | `APP_AUTO_BRAKE_ENTER_ERROR_MPS` / `APP_AUTO_BRAKE_ENTER_RATIO` |
| 前进刹车释放误差 | max(0.05 m/s, 0.02×目标) | `APP_AUTO_BRAKE_RELEASE_ERROR_MPS` / `APP_AUTO_BRAKE_RELEASE_RATIO` |
| 滑行评估 / 预算 | 200 / 600 ms | 小超速先观察滑行；预算到期只申请刹车，不代表故障 |
| 可用滑行减速趋势 | 0.05 m/s² | `APP_AUTO_A_PROGRESS_MPS2` |
| 刹车释放预测延迟 | 80 ms | `APP_AUTO_BRAKE_RELEASE_DELAY_MS` |
| 动作确认 / 完整负向资格 / 中位驻留 | 300 / 100 / 100 ms | `APP_AUTO_ACTION_ACK_MS` / `APP_AUTO_QUALIFY_MS` / `APP_AUTO_NEUTRAL_DWELL_MS` |

以上为当前代码配置。PID输出由模式二门控投影到当前动作允许的物理区间；普通AUTO推进和F→R资格制动均使用1000/1500/2000 µs定义的物理输入范围。

## 电气与采集

| 项目 | 数值 / 接口 | 依据 |
| --- | --- | --- |
| MCU / ESC | STM32F407VET6 / EZRUN MAX5 HV G2，非 Plus | 原工程 / 实物铭牌 |
| 电调适配模式 | 2：前进、刹车、倒车 | 实车确认F→R需刹车/回中/再次负向，R→F可持续正向由刹车进入前进 |
| ESC 持续 / 峰值电流 | 250 / 1600 A | 手册额定能力 |
| 电调内置 BEC 持续 / 瞬时电流 | 8 / 25 A | 官方手册「03 产品特色」 |
| 电调内置 BEC 可选输出电压 | 6 / 7.4 / 8.4 V | 官方手册「04 产品规格」 |
| 电调动力电池组合 | 两块 4S 串联，共 8S | 用户提供 |
| 舵机供电评估负载 | 2 × 120 kg 级、3 × 27 kg 级舵机 | 扭矩标称不作为电流参数 |
| 电调内置 BEC 实际设置（V） | | |
| 舵机实际供电来源 / 电压（V） | | |
| 电池电压（V）/ 容量（Ah）/ 电机 KV | | |
| OTA 刹车 / 倒车 / 拖刹比例（%） | | |
| 上位机 UART | UART4：PC10 TX、PC11 RX；115200 / 8N1 | `Core/Src/usart.c` |
| 原调试 UART | USART1：PA9 TX；115200 / 8N1 | 基线仅配置发送 |
| ESC 扩展 RX | PD15 GPIO/EXTI + TIM5；115200 / 8N1 | 实板已观察合法FE32与raw RPM随油门更新；伪起始位非致命过滤 |
| ESC 信号电平（V） | | |
| ESC / 转向 PWM | PC6 / TIM8 CH1；PC7 / TIM8 CH2 | `servo_basic_output.c` |
| RC 油门 / 转向 | PD12 / TIM4 CH1；PD13 / TIM4 CH2 | 既有输入捕获路径 |
| RC有效脉宽 / 油门输出 | 900–2100 µs | 通过合法性与毛刺确认后原样输出，不套用自动推进端点 |
| 未使用引脚 | PD14 | 实物确认未接线；固件不配置TIM4 CH3，不存在guard输入 |
| OLED页面开关 | PD3；输入上拉 | 拨动开关，低电平运行页、高电平诊断页；40 ms消抖，不使用EXTI |
| 电池 ADC | PC2 / ADC1_IN12 | 原上行电压来源 |
| ADC 换算 | `raw / 4095 × 3.3 × 11` V | 原参考电压与换算倍率，不代表电池额定电压 |
| Hall A / B | PE13 / PE14；上拉、下降沿 | 基线计数选择 B 通道 |
| Hall 分辨率 / 最小脉冲间隔 | 10 次/转 / 1500 µs | 用于独立周期测速与脉冲诊断；上行回传，不进入 ESC 主速度闭环 |
| Hall 毛刺确认次数 | 3 | `APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS=3` |

[电调官方手册](https://www.hobbywing.com/uploads/file/20230512/1c306ab68a1aa3f57bd1dc6fa3bc7c2e.pdf)给出额定能力；额定电流不是软件出力上限。BEC 指电调内部的低压稳压电源，不代表另装独立模块；其供电能力与本车实际供电接线分别记录。

## 时序与新增参数

| 参数 | 单位 | 数值 | 来源 / 含义 |
| --- | --- | --- | --- |
| 原控制周期 | ms | 20 | `ServoBasic_Task` |
| 状态回传周期 | ms | 50 | `RobotDataTransmitTask`：20 Hz |
| 上位机命令超时 | ms | 250 | `APP_ORIN_PWM_TIMEOUT_DEFAULT_MS` |
| RC 信号超时 | ms | 100 | `APP_RC_SIGNAL_TIMEOUT_MS` |
| 电机极对数 p | 对 | | 仅用于物理模型复核，不作为当前低档速度输出前提 |
| 波箱型号 / 所选档位 | | 固定低档 | 用户确认当前运行不切档 |
| 波箱一档 / 二档比 G档 | 倍 | | 波箱输入 RPM / 输出 RPM |
| 轮端减速比 G轮 | 倍 | | 波箱输出至车轮之间各级减速的乘积 |
| 实测轮轴 RPM / rpm_raw，低档 / 第二组参考 | | **0.14115** / 0.3147951 | 当前固件启用低档0.14115；第二组只保留参考，不参与运行 |
| 有条件总机械比，第一档 / 第二档 | 倍 | 35.42 / 15.88；严格窗 35.70 / 15.87 | 仅在 `raw×10=eRPM`、2 极对、Hall 10 脉冲/轮轴圈成立时；36/16 为候选，未确认 |
| ESC 电脑接收帧率 | Hz | 约 10 / 50 | 2026-09-15、17 USB-TTL 实测；是收帧频率，不证明内部传感器采样率；C63A上行50ms快照不等于完整事件日志 |
| ESC 上行速度新鲜超时 | ms | 250 | 独立观测门控；静止约10 Hz、油门约50 Hz实测 |
| FE32 byte11动作状态 | | 0中位 / 1驱动 / 2刹车 | 前后驱动均为1；状态只区分动作，不直接给方向 |
| RC方向确认样本 | 个不同FE32样本 | 启动2 / 换向1 | 启动需连续DRIVE；已见BRAKE/NEUTRAL或相反侧低速证据后的新DRIVE直接确认新方向；RC按FE32事件顺序消费 |
| RC中位学习窗口 | 个合格样本 | 5 | 仅NEUTRAL、RPM=0且PWM稳定样本，取滚动中位数，不写Flash |
| AUTO输出中点 / 前进端点 / 负向端点 | µs | 1500 / 2000 / 1000 | `P0 / P_F / P_R`；完整物理输入范围 |
| F→R完整负向资格 | µs / ms | 1000 / 100 | 最终实际PWM到达`P_R`后开始计时，需新的BRAKE反馈和停稳证据 |
| R→F策略 | | 中位滑停后正向启动 | 当前AUTO不输出正向刹车；倒车未停稳时不允许`P>P0` |
| 停稳阈值 / 样本数 / 覆盖时长 | m/s / 次 / ms | 0.05 / 3 / 150 | 不复用旧 Hall 无脉冲超时作为新停稳条件 |
| 回中驻留 | ms | 100 | F→R倒车许可和未知状态启动前的中位确认 |

传动换算先到轮轴：当前低档`wheel_rpm = rpm_raw × 0.14115`；轮胎半径115 mm独立参与线速度计算。两组原始记录和审计见工作区`资料/实测记录/2026-09-17_Windows_ESC直读与两档比例/`。极对数和拆分齿比可用于以后复核，不作为当前速度幅值输出的前提。AUTO首次启动和换向使用各自阶段的新停稳、中位及动作证据；正常行驶中的速度调节不要求车辆保持停稳。

运行参数集中在`app_vehicle_config.h`的`APP_ESC_*`、`APP_SPEED_PID_*`和`APP_AUTO_*`数值中。配置有效性全部由数值范围和相互关系推导，不再使用人工valid开关。中点和`B_min`在AUTO启用或未确认停稳时保持上一份有效配置，避免运行中改变连续制动会话的电气含义；PID增益变更会清I/D动态状态。
