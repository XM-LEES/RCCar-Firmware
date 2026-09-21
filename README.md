# RC 线控底盘

基于 STM32F407VET6，接收目标速度与前轮转角，控制 RC 底盘的电调和转向舵机。项目以 `feature/ackermann-chassis` 为基线，保留有效功能和车辆参数，清理未安装设备的占位接口。

电调适配 MAX5 HV G2 模式二，处理前进、主动减速、停稳与双向换向。主车速幅值由实测低档传动系数先把`rpm_raw`换算到轮轴RPM，再结合独立轮胎半径计算；`esc_motion_estimator`、`longitudinal_controller`和`mode2_drive_gate`分别负责观测、速度控制与电调动作许可。模式二门控对两个方向使用独立的刹车力度、保持时间和最终PWM达标阈值，只有“连续足量刹车→新停稳证据→回中驻留”完成后才允许相反方向推进。回中后不会自动补发更强的同侧信号，因为该信号可能已经被电调解释为推进。

状态帧额外上传下位机按脉冲周期计算的 Hall 速度，供上位机独立对比；Hall 累计计数仍保留在固件内部。RC 下的 Hall 方向采用同样的对称模式二序列，避免把刹车误报为反向运动。FE32 帧已通过电脑采集与原始数据回放；前进、后退分段直读确认RPM只提供幅值，没有可靠方向位。C63A A3 链路实测持续 FE/NE，PD15 EXTI + TIM5 软件UART已在实板观察到合法帧计数和raw RPM随油门变化；半位确认过滤出的伪起始位不再作为致命接收错误。闭环控制效果仍待验证。

低档上行速度采用实测`0.14115轮轴RPM/rpm_raw`和现有115 mm轮胎半径。停稳、跟踪刹车和模式二双向参数均从实际数值推导有效性，不再依赖人工valid开关。非零速度目标不经过软件死区或速度上限，最终输出只受前馈表端点、PI修正范围和PWM硬边界约束。换向或受控停车需要建立明确模式二状态时，刹车使用1000/2000 µs完整行程；命令变号会立即清除旧方向的速度斜坡。

RC油门在脉宽有效性与毛刺确认通过后保留接收机完整行程，不套用自动推进的标定端点。AUTO下Hall方向来自模式二已经确认的实际方向，刹车期间维持原符号，停稳或方向未知时归零。

实物确认PD14没有外部接线，固件不再配置TIM4 CH3，也不存在guard或第三路RC输入。RC接管只使用PD12油门和PD13转向。

板载PD3拨动开关选择OLED页面：低电平为运行页，显示控制模式、实时`E/S`、`SP/ST`、电压和Hall；高电平为诊断页，同样固定显示实时`E/S`，并显示`U/X`接收诊断、`M/Q`模式二状态/原因、有效性位、ESC/Hall速度以及FE32帧/raw RPM。开关轮询不占用EXTI；即使实物SW电平不变化，当前可见页面也始终保留`E/S`。

## 架构

[![系统架构](docs/diagrams/system.svg)](docs/ARCHITECTURE.md)

- [架构与速度闭环](docs/ARCHITECTURE.md)：模块、数据流与控制规则。
- [接口约束](docs/INTERFACES.md)：串口、控制源与状态语义。
- [车辆数据](docs/VEHICLE.md)：源码参数、硬件数据与标定来源。
- [draw.io 图源](docs/diagrams/chassis.drawio)：系统架构、速度环两页。

## 工程入口

| 用途 | 入口 |
| --- | --- |
| ARM GCC 构建 | `CMakeLists.txt`、`cmake/arm-none-eabi-gcc.cmake` |
| Keil 工程 | `MDK-ARM/WHEELTEC.uvprojx`，ARMCC 5.06 update 7；Windows 归档中的同源功能版本已完整编译，0 错误、0 警告 |
| 主机测试 | `tests/host/`；真实 FE32 回放验证原始文件的 SHA-256 |
| 静态接口验收 | `tools/acceptance/check_firmware_contract.py` |
| GCC 向量表验收 | `tools/acceptance/check_firmware_vectors.py` |
| Fire-Debugger 下载配置 | `tools/openocd/c63a_fire_dap.cfg`，CMSIS-DAP/HID + SWD |

以下命令在本仓库目录运行。

```sh
cmake -S . -B build/host
cmake --build build/host
ctest --test-dir build/host --output-on-failure
python3 -B tools/acceptance/check_firmware_contract.py --workspace-root .
```

真实回放默认读取工作区的 `../资料/实测记录/`。独立检出本仓库时，可用 `RCCAR_ESC_FE32_REPLAY_ROOT` 指向完整采集目录；缺失或校验不符会使回放失败。显式设置 `RCCAR_ENABLE_ESC_FE32_REPLAY=OFF` 只运行单元测试，不代表已经验证真实数据回放。

```sh
cmake -S . -B build/arm \
  -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi-gcc.cmake \
  -DCMAKE_BUILD_TYPE=Debug \
  -DRCCAR_BUILD_HOST_TESTS=OFF -DRCCAR_BUILD_FIRMWARE=ON
cmake --build build/arm
python3 -B tools/acceptance/check_firmware_vectors.py build/arm/WHEELTEC.elf
```

产物为 `build/arm/WHEELTEC.elf`、`.hex`、`.bin` 和 `.map`，按 STM32F407VET6 的 512 KiB Flash 链接。Debug ELF 保留源码和变量类型信息，供联调查看状态。CMSIS-DAP 下载器可在 Linux 使用 OpenOCD；具体 SWD 接线和主控连通性须另行验证。
