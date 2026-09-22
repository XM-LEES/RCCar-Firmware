# ESC观测与速度发布

本文定义ESC编程口反馈如何进入固件、如何形成RC下的有符号速度、哪些信息只用于显示、哪些信息能参与控制。模块总览见[ARCHITECTURE.md](ARCHITECTURE.md)，24字节报文见[INTERFACES.md](INTERFACES.md)。

![ESC观测链路](diagrams/esc-observation.svg)

## 设计边界

ESC FE32帧的RPM是无符号幅值。byte 11只表示动作：`NEUTRAL`、`DRIVE`、`BRAKE`。它不直接表示车辆前进或后退。

固件把四类信息分开处理：

| 类别 | 含义 | 处理规则 |
| --- | --- | --- |
| 幅值 | `rpm_raw`按低档实测系数换算出的车速绝对值 | 新鲜、RPM有效、换算配置有效时成立；与方向有效性分离 |
| 方向 | 车辆物理运动方向，前进为`+1`，倒车为`-1` | RC下由动作、软件PWM上下文和运动证据推断；AUTO下由模式二控制状态维护 |
| 动作 | ESC报告的`NEUTRAL / DRIVE / BRAKE` | 用于颜色、诊断、RC方向证据和AUTO门控，不等价于方向 |
| 安全状态 | 反馈是否新鲜、是否停稳、是否RX错误、AUTO是否授权 | 不用显示层处理或方向猜测改写 |

方向未知时，不伪造符号。固件仍可上传正的速度幅值并清方向可信位；上游必须按有效位解释，不能把无效符号补成零或猜成前进。

## 数据与时序

ESC观测链路采用三个更新节拍；MCU内部使用本地时间，未与ESC内部采样时钟同步：

| 更新节拍 | 当前作用 | 约束 |
| --- | --- | --- |
| ESC发送节奏 | 实测空闲约10Hz，运行约50Hz | 由电调决定，固件不控制 |
| C63A控制周期 | 20ms绝对周期 | 负责输出PWM、AUTO闭环、快照发布；新FE32通知不能移动该周期 |
| 状态上行周期 | 50ms | 只发送当前快照，不是完整事件日志 |

PD15软件UART在每个收到的字节上记录MCU接收时间和紧凑的软件PWM上下文。该上下文打包为32bit：低12bit为PWM脉宽，bit12为RC标志，高19bit为控制源epoch。控制源epoch只在RC/AUTO来源切换时递增；同一RC来源下PWM变化会更新脉宽，但不切断source。上下文描述“字节到达C63A时的软件命令环境”，不是ESC内部真实采样时刻，也不是PWM硬件输出波形的回读。

FE32解析成功后，样本进入两条路径：

| 路径 | 使用者 | 语义 |
| --- | --- | --- |
| 最新快照 | AUTO、诊断、通用幅值估计 | 保留最后一份合法FE32；AUTO仍按20ms周期读取最新样本 |
| RC事件队列 | RC方向观测 | `EscTelemetryObservedSample_t`固定8深度队列保存完整样本和接收上下文；Servo任务收到通知后按顺序消费 |

RC队列只改变观测证据的保留方式，不改变RC油门直通频率，也不让控制器在FE32到达时额外运行PI或写PWM。Servo任务仍然拥有观测状态，避免多个任务同时修改方向状态。

FE32帧的PWM上下文取首字节接收完成时的软件PWM标记；样本接收时间取末字节接收完成时刻，用于年龄计算。完整32字节帧必须属于同一个控制源epoch，否则该帧的RC观测上下文无效。同一RC来源下，帧传输中途的PWM变化不会使该帧失效，帧仍使用首字节上下文；这样排除首字节接收后才写入的命令对该帧的解释。非RC上下文不会进入RC观测队列。

控制源切换时，Servo层先清除旧边界证据，再发布新source的输出上下文。随后到达的样本必须与当前source匹配，才能进入RC方向观测。

一次Servo任务执行遵循两个入口：

| 入口 | 执行内容 | 不执行内容 |
| --- | --- | --- |
| FE32通知 | `ServoBasic_ProcessEscObservation()`顺序消费RC队列，更新RC方向观测和速度发布快照 | 不更新RC输入，不运行AUTO闭环，不写新的PWM |
| 20ms控制deadline | 先消费已到达的RC反馈，再更新RC/AUTO控制源、计算并写入PWM、更新AUTO状态、发布控制快照 | 不补跑过去错过的控制周期 |

如果一次通知中队列已有多份FE32，固件按接收顺序处理观测证据，只发布最后一份处理后的相干结果。相干结果表示幅值、方向、动作、有效性和时间来自同一批已处理进度，不发布“新RPM加旧未完成方向”的半状态。

接收epoch变化、delivery epoch变化、RX重启、严重接收错误、source切换或队列溢出都是证据边界。边界之后，旧BRAKE、旧NEUTRAL和旧PWM上下文不能继续作为新一代样本的换向证据。正常静止时约100ms的FE32间隔不是丢帧；只要没有超过遥测新鲜超时，就不因没有新帧而制造故障。

实现使用固定容量队列和固定内存，不在接收、解析或Servo观测路径分配动态内存。接收层诊断包括RC观测队列入队、消费、丢弃、溢出、上下文拒绝、source边界和队列峰值，通过`EscTelemetry_GetDiagnostics()`读取。Servo层诊断包括样本过期、source不匹配、delivery gap、样本年龄、批处理耗时、批处理样本数和20ms deadline迟到情况，通过`ServoBasic_GetObservationDiagnostics()`读取。这些诊断用于区分“没有样本”“样本晚到”和“样本被边界切断”，不是24字节上行协议的一部分。`max_publish_age_ms`只记录每个sample/epoch第一次发布时从接收到发布的年龄，重复空闲快照不会放大该指标。

`servo_basic_control_snapshot_t.control_tick_ms`记录本次控制快照的发布时间。FE32事件路径也会发布快照，所以该字段不能解释为最近一次PWM实际变更时间。

## RC方向状态

符号定义：

| 符号 | 定义 |
| --- | --- |
| `D` | 已确认方向：未知、前进、倒车 |
| `K` | 对外方向是否有效 |
| `N` | 学到的RC中位PWM |
| `q` | 当前样本的软件PWM上下文相对`N`的侧向：高于中位为前进侧，低于中位为倒车/前进制动侧，等于中位为0 |
| `M` | 同一FE32样本具有可用幅值，且换算速度超过停稳阈值 |
| `H` | 已确认方向之后见过非DRIVE动作，或见过相反侧DRIVE但没有可用运动证据；它允许下一次相反侧DRIVE重新判向 |

中位学习只接受新鲜`NEUTRAL`、`rpm_valid=1`、`rpm_raw=0`、PWM非零的样本。两份不同FE32样本的PWM必须是同一个整数脉宽，才形成一个中位候选；保存最近最多5个候选，取排序后的中位数作为`N`。中位不写Flash；退出RC或遥测失效不删除已学中位。

### DRIVE判定表

下表只针对RC生效、新鲜FE32、同一receive epoch内的新样本。入口无效、RC退出、遥测过期或未知动作会使方向输出无效；已学中位可保留。

| 条件 | 结果 |
| --- | --- |
| `S=NEUTRAL` | 若`D`已知，保持`D`并置`H=1`；若满足中位学习条件则更新`N` |
| `S=BRAKE` | 若`D`已知，保持`D`并置`H=1`；不能单独建立方向 |
| `S=DRIVE`且`N`未知 | 不建立方向，`K=0` |
| `S=DRIVE`且`q=0` | 中位PWM不提供方向证据；若`D`已知则继续输出`D`，不清`H` |
| `S=DRIVE`且`M=0` | 不翻向；`M=0`只表示没有可用运动证据，可能来自低速、RPM无效或换算不可用；若`q`与`D`相反则置`H=1` |
| `D`未知，连续两个不同样本满足同一`q`且`M=1` | 确认`D=q` |
| `D`已知，`q=D`且`M=1` | 保持`D`，清`H` |
| `D`已知，`q!=D`且`H=0` | 保持旧`D`，等待非DRIVE或无运动证据的相反侧DRIVE |
| `D`已知，`q!=D`且`H=1`且`M=1` | 确认换向，`D=q`，清`H` |

这个状态机反映实车观察到的模式二行为：前进转倒车必须经历刹车/回中/再次负向；倒车转前进可在持续正向输入下由刹车进入前进。固件不计算电调内部刹车阈值，只根据FE32动作和随后的新DRIVE样本按这些证据推定方向。

## 速度发布

当前低档速度幅值为：

```text
wheel_rpm = rpm_raw * 0.14115
speed_magnitude_mps = wheel_rpm * 2*pi*0.115 / 60
```

传动换算到轮轴为止。轮胎半径独立配置；更换轮胎时改半径，不重拟合轮轴系数。

24字节上行的ESC速度遵循下表：

| 条件 | 字节7-8 | bit6幅值有效 | bit12停稳 | bit23方向已知 |
| --- | --- | --- | --- | --- |
| 已确认停稳 | `0` | 0 | 1 | 0 |
| 幅值有效且方向已知 | 有符号速度，前进正、倒车负 | 1 | 0 | 1 |
| 幅值有效但方向未知 | 正的速度幅值 | 1 | 0 | 0 |
| 幅值无效 | `0`占位 | 0 | 0 | 取决于内部方向状态，不可单独使用 |

Dashboard和其他上游消费者必须同时看数值和有效位。bit6清零表示ESC速度幅值不可用；bit23清零表示字节7-8不能作为有符号速度；bit12置位表示固件确认停稳，速度字段清零。

显示连续性、控制安全和传感器融合的职责分开：

| 层 | 可做的事 | 禁止的事 |
| --- | --- | --- |
| 固件观测 | 新鲜且RPM/配置有效时发布幅值；新样本RPM无效或换算不可用时立即清幅值有效性；中位PWM不打断已确认方向 | 不能为美化曲线伪造方向；不能延长新鲜超时 |
| Dashboard显示 | 当前按固件有效位显示；方向或幅值无效时显示为空白/无效 | 不能把invalid改写成真实零速；不能把显示结果回写控制 |
| 控制与安全 | 只使用固件有效性、停稳证据、模式二门控和故障位 | 不能依赖Dashboard显示形态；不能把方向未知当成已确认方向 |
| 里程计/融合 | 可融合ESC幅值、Hall、IMU或其他外部源 | 方向未知时必须保持不确定性，不能用单传感器幅值构造带符号位移 |

RC下，Hall方向提示使用同一个`D`。方向未知或ESC确认停稳时传给Hall的方向为0。Hall本身仍按脉冲周期独立计算速度，换向后需要新的脉冲对重新建立周期。

## RC与AUTO边界

| 项目 | RC | AUTO |
| --- | --- | --- |
| 输出来源 | 接收机油门和转向经合法性、毛刺处理后直通 | 串口速度/转角经过纵向控制器和模式二门控 |
| FE32消费 | RC事件队列按样本顺序更新方向观测 | 20ms周期读取最新FE32快照 |
| 方向状态 | RC方向观测器维护`D/K/H/N` | AUTO的模式二门控和车辆方向状态独立维护 |
| 速度符号 | 使用RC观测方向 | 使用AUTO内部方向 |
| 对执行器的反向影响 | 观测结果不改变RC输出 | AUTO方向和停稳证据参与控制决策 |
| 切换边界 | 进入/退出RC清方向输出和换向证据，保留中位学习；source切换后旧RC队列证据失效 | RC介入使AUTO执行历史失效，退出后AUTO重新建立证据 |

RC和AUTO共用PD15接收、FE32解析、幅值换算、停稳阈值和24字节出口。两者不共用方向状态。RC修复不能被描述为AUTO换向修复；AUTO仍按独立门控规则处理F->R和R->F。

## 故障与级联影响

方向短时未知不是接收故障、不是急停、不是RC输出限制。它的直接影响是：bit23清零，有符号ESC曲线无效，角速度不使用ESC符号速度。

可能触发安全语义的是这些条件：

| 条件 | 影响 |
| --- | --- |
| PD15停止位错误、采样迟到、缓冲溢出等RX错误 | 记录接收错误，建立receive epoch边界，清旧证据 |
| FE32超过新鲜超时 | 幅值不可用；AUTO反馈不可用；RC方向输出无效 |
| RPM字段无效或换算配置无效 | 幅值不可用；不能形成新的运动证据 |
| 停稳证据成立 | 上行速度清零，bit12置位；AUTO可把它用于模式二步骤 |
| 队列溢出、delivery gap或source变化 | 旧方向过渡证据失效；最新快照路径仍按当前样本有效位和新鲜度解释 |

因此，显示层看到曲线空白不应自动解释为急停、掉线或车辆真的为零速。只有对应的状态位和故障计数能说明原因。

## 验收边界

主机测试能验证给定样本序列、队列边界和状态转移是否按表执行。它不能证明真实电调内部状态，也不能替代实车RC测试。

当前硬件基线来自实车观察：RC前进收油、前进刹车、倒车收油、倒车转前进时，ESC速度幅值方向正确；旧实现中过中位PWM会造成短暂有幅值但方向无效的空白。修复要求是在FE32新鲜、幅值有效且方向已建立时，不因中位PWM本身打断ESC有符号速度。真正丢失遥测、RPM无效、receive epoch变化或队列溢出时，仍必须暴露无效语义。

任何改变RC观测、FE32调度、24字节有效位或Dashboard解释规则的提交，都需要重新跑主机检查，并由实车验证RC速度方向连续性。固定队列和任务通知降低调度抖动造成的信息丢失，但文档不把它表述为MCU最坏情况实时性能保证。

## 实现入口

| 职责 | 文件 |
| --- | --- |
| PD15软件UART与STM32服务入口 | [esc_soft_uart_stm32.c](../WHEELTEC_APP/esc_soft_uart_stm32.c)、[esc_telemetry_stm32.c](../WHEELTEC_APP/esc_telemetry_stm32.c) |
| FE32解析、receive/delivery epoch、最新快照和RC样本队列 | [esc_telemetry.c](../WHEELTEC_APP/esc_telemetry.c)、[esc_telemetry.h](../WHEELTEC_APP/Inc/esc_telemetry.h)：`EscTelemetryObservedSample_t`、`EscTelemetry_PopSample()`、`EscTelemetry_DiscardSamples()` |
| RC方向状态机 | [rc_direction_observer.c](../WHEELTEC_APP/rc_direction_observer.c)、[rc_direction_observer.h](../WHEELTEC_APP/Inc/rc_direction_observer.h) |
| Servo任务、RC/AUTO边界、速度快照 | [servo_basic_control.c](../WHEELTEC_APP/servo_basic_control.c)、[servo_basic_control.h](../WHEELTEC_APP/Inc/servo_basic_control.h)：`ServoBasic_ProcessEscObservation()`、`ServoBasic_GetObservationDiagnostics()` |
| RPM幅值、停稳证据 | [esc_motion_estimator.c](../WHEELTEC_APP/esc_motion_estimator.c)、[esc_motion_estimator.h](../WHEELTEC_APP/Inc/esc_motion_estimator.h) |
| 24字节状态封装 | [data_task.c](../WHEELTEC_APP/data_task.c) |
| 参数来源 | [app_vehicle_config.h](../WHEELTEC_APP/Inc/app_vehicle_config.h) |
