# 车辆数据

数值以 `feature/ackermann-chassis@b4ee219` 的实际源码为基线。下表记录代码配置值及其单位，不将配套说明中的不同数值覆盖源码；空白项不表示零值。

主要来源：[车辆配置](https://github.com/XM-LEES/RCCar-Firmware/blob/b4ee2191b6722500a94b4a65ce5e85bbfc94b607/WHEELTEC_APP/Inc/app_vehicle_config.h)、[Hall 参数](https://github.com/XM-LEES/RCCar-Firmware/blob/b4ee2191b6722500a94b4a65ce5e85bbfc94b607/WHEELTEC_APP/hall_speed.c)、[控制与前馈标定](https://github.com/XM-LEES/RCCar-Firmware/blob/b4ee2191b6722500a94b4a65ce5e85bbfc94b607/WHEELTEC_APP/servo_basic_control.c)。源码中的其余有效配置同样沿用，不因未列入本表而取消。

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

## 输出与推进标定

| 参数 | 代码配置值 | 含义 |
| --- | --- | --- |
| PWM 周期 / tick | 2631 / 1 µs | TIM8：PSC=167、ARR=2630 |
| ESC 脉宽输入范围 / 中点 | 1000–2000 / 1500 µs | 全输入范围与中点 |
| ESC 前进 / 倒车起点参数 | 1560 / 1440 µs | 原参数，不代替前馈表中的低速节点 |
| ESC 前进上限 / 倒车下限 | 1650 / 1350 µs | 基线最终输出软限幅，区别于全输入端点 |
| 转向中点 / 幅度 | 1500 / ±395 µs | 有效范围 1105–1895 µs |
| 转向角—脉宽 | −0.349 / 0 / +0.349 rad ↔ 1105 / 1500 / 1895 µs | 正左转对应 PWM 增大 |
| 速度目标处理 | 无缩放、无软件死区、无速度上限 | 精确零表示停车；非零目标由前馈表端点与最终PWM边界自然饱和 |
| 参考速度变化率 | 4.000 m/s² | `APP_ORIN_ACCEL_LIMIT_MMPS2=4000` |
| 转角变化率 | 0.900 rad/s | `APP_ORIN_STEERING_RATE_LIMIT_MRADPS=900` |
| PI 开关 / Kp / Ki | 1 / 20.0 / 0.0 | Kp：µs/(m/s)；Ki：µs/[(m/s)·s] |
| PI 微调幅度 | ±12 µs | 在前馈脉宽上叠加，不是全量 PWM |
| 前馈标定表 | `s_orin_forward_ff_table` / `s_orin_reverse_ff_table` | 沿用原速度—脉宽节点，分段线性插值 |

以上为基线代码值。推进曲线、PI 增益在模式二下的适用性未验证；新增刹车边界单独记录，不以旧倒车限幅代替。

## 电气与采集

| 项目 | 数值 / 接口 | 依据 |
| --- | --- | --- |
| MCU / ESC | STM32F407VET6 / EZRUN MAX5 HV G2，非 Plus | 原工程 / 实物铭牌 |
| 电调适配模式 | 2：前进、刹车、倒车 | 本车模式二动作未验证 |
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
| RC AUX | PD14 / TIM4 CH3 | 保留PWM输入捕获；真实用途待确认，当前不参与接管、停止或故障判断 |
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
| ESC 电脑接收帧率 | Hz | 约 10 / 50 | 2026-09-15、17 USB-TTL 实测；是收帧频率，不证明内部传感器采样率 |
| ESC 上行速度新鲜超时 | ms | 250 | 独立观测门控；静止约10 Hz、油门约50 Hz实测 |
| 制动映射中点 / F→R满请求端点 | µs | 1500 / 1350 | F→R刹车端点小于中点，独立于倒车推进限幅 |
| R→F满请求端点 | µs | 1650 | 大于中点，独立于前进推进限幅 |
| 刹车增益 Kb / 力度上限 | 1/(m/s) / 0…1 | 0.50 / 0.70 | 归一化制动请求，不表示测得的制动力 |
| 刹车进入 / 释放误差 | m/s | 0.20 / 0.10 | 相对参考速度的误差滞回；这是唯一速度误差刹车逻辑 |
| 停稳阈值 / 样本数 / 覆盖时长 | m/s / 次 / ms | 0.05 / 3 / 150 | 不复用旧 Hall 无脉冲超时作为新停稳条件 |
| F→R / R→F刹车请求 | 0…1 | 0.70 / 0.70 | 两个方向保留独立运行参数 |
| F→R / R→F最短保持 | ms | 100 / 100 | 最终PWM连续达标的最短时间 |
| F→R / R→F资格差值 | µs | 100 / 100 | 分别为`center-PWM`和`PWM-center`；使用最终实际PWM |
| 回中驻留 | ms | 100 | 两个方向共用；机械响应不设固定完成超时 |

传动换算先到轮轴：当前低档`wheel_rpm = rpm_raw × 0.14115`；轮胎半径115 mm独立参与线速度计算。两组原始记录和审计见工作区`资料/实测记录/2026-09-17_Windows_ESC直读与两档比例/`。极对数和拆分齿比可用于以后复核，但不再阻止当前上行速度幅值。自动推进仍要求停稳、模式二时序、制动映射等配置有效。

运行参数集中在`app_vehicle_config.h`的`APP_ESC_*`和`APP_MODE2_*`数值中。配置有效性全部由数值范围和相互关系推导，不再使用人工valid开关。`APP_MODE2_FWD_TO_REV_*`与`APP_MODE2_REV_TO_FWD_*`保留两个方向的独立调节能力。
