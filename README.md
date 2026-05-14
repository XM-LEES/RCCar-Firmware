# RCCar-new 下位机固件

`RCCar-new/` 是整车接管工作区里的 STM32F407 底盘固件仓库。系统级设计看根目录 `docs/`，本仓库负责把下位机代码结构、数据流、功能边界、提交规则和调试产物落点说明清楚。

系统级阶段目标、架构、传感器接入、ROS topic、UART4 协议字段、测试记录和上机验收以根目录文档为准：

- `../docs/阶段路线图.md`
- `../docs/系统架构与数据流.md`
- `../docs/接口与协议.md`
- `../docs/开发流程与验证规范.md`

本仓库文档负责细化下位机实现，不另起一套系统级设计。代码、子仓库文档和根文档冲突时，先按当前代码事实核对，再同步修正文档。

## 当前职责

- 接收 UART4 Ackermann command。
- 执行 `speed_mps + steering_angle_rad`。
- 输出 ESC PWM 和前轮转向舵机 PWM。
- 采集霍尔计数并提供速度反馈。
- 提供轻量 speed PI trim、battery telemetry、RC 接管、急停、刹车和通信超时停车。

## 当前实现锚点

- Keil 工程：`MDK-ARM/WHEELTEC.uvprojx`
- STM32 启动和外设初始化：`Core/Src/main.c`
- FreeRTOS 任务创建：`Core/Src/freertos.c`
- UART4 command 解析：`WHEELTEC_APP/SerialControl_task.c`
- Ackermann 执行和安全仲裁：`WHEELTEC_APP/servo_basic_control.c`
- PWM 输出：`WHEELTEC_APP/servo_basic_output.c`
- RC 输入捕获：`WHEELTEC_APP/servo_rc_capture.c`
- 霍尔测速：`WHEELTEC_APP/hall_speed.c`
- UART4 telemetry：`WHEELTEC_APP/data_task.c`
- 固件契约静态检查：`tools/acceptance/check_firmware_contract.py`
- 本地实现摘要：`docs/current-state.md`
- 本地下位机流程：`docs/开发流程与验证规范.md`

## 当前关键边界

- 正式自动控制入口只接受 `cmd=0x01` 的 Ackermann command。
- UART4 下行/上行帧格式必须服从根 `docs/接口与协议.md`。
- Ubuntu 开发环境只做静态检查；固件编译、烧录和 Keil 级调试由用户切换 Windows/Keil 后完成。
- 历史 CAN、USART3/RS485、Bluetooth/App、USB HID、Ranger、AutoRecharge、ICM20948/IMU、RGB APP 等内容不属于当前主路径。

## 文档目录

`docs/` 不再设置单独 README/索引入口；本文件就是 `RCCar-new/` 的唯一文档入口。

- `docs/current-state.md`：代码结构层级、任务/中断、数据流、功能边界和调试入口。
- `docs/开发流程与验证规范.md`：提交切分、必读范围、强制一致性收尾检查、测试记录和日志落点。
- `docs/test-records/`：可提交的 Markdown 测试记录。
- `docs/vendor/`：厂家原始资料。
- `tools/acceptance/`：下位机静态契约检查和后续 PC 串口验收工具。
- `logs/`：原始日志和临时调试产物，默认不提交。
