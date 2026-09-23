# RC 线控底盘

基于 STM32F407VET6，接收目标速度与前轮转角，控制 RC 底盘的电调和转向舵机。项目以 `feature/ackermann-chassis` 为基线，保留有效功能和车辆参数，清理未安装设备的占位接口。

电调适配 MAX5 HV G2 模式二。主车速幅值由实测低档传动系数先把`rpm_raw`换算到轮轴RPM，再结合独立轮胎半径计算；Hall速度按脉冲周期独立计算。RC遥控器控制下，两路速度共用FE32动作反馈确认的物理方向。串口自动闭环继续使用独立的自动控制状态机。

FE32帧已通过电脑采集与原始数据回放；RPM只提供幅值，byte 11表示`NEUTRAL / DRIVE / BRAKE`动作而不表示方向。实车确认模式二换向非对称：F→R需要刹车、回中和再次负向输入，R→F可在持续正向输入下由刹车直接进入前进。RC方向观测和AUTO换向门控都以FE32动作和本源输出上下文建立许可。

低档上行速度采用实测`0.14115轮轴RPM/rpm_raw`和现有115 mm轮胎半径。停稳、跟踪刹车和模式二参数均从实际数值关系推导有效性。AUTO纵向控制使用完整PID输出，当前工程初值为`Kp/Ki/Kd=120/20/0`；最终输出受1000/1500/2000 µs物理端点和当前模式二动作许可约束。

RC方向观测器在首次运动时用连续`DRIVE`样本确认方向；制动开始时即使PWM已经换边而FE32仍短暂保留旧`DRIVE`，也保持原物理运动方向和连续ESC速度。看到`BRAKE/NEUTRAL`后的新`DRIVE`直接确认新方向。Hall与ESC共用这一结果，RC油门输出保持直通。

实物确认PD14没有外部接线，固件不再配置TIM4 CH3，也不存在guard或第三路RC输入。RC接管只使用PD12油门和PD13转向。

板载PD3拨动开关选择OLED页面：低电平为运行页，显示控制模式、实时`E/S`、`SP/ST`、电压和Hall；高电平为诊断页，同样固定显示实时`E/S`，并显示`U/X`接收诊断、`M/Q`模式二状态/原因、有效性位、ESC/Hall速度以及FE32帧/raw RPM。开关轮询不占用EXTI；即使实物SW电平不变化，当前可见页面也始终保留`E/S`。

## 架构

- [架构与速度闭环](docs/ARCHITECTURE.md)：模块、数据流与控制规则。
- [AUTO纵向控制实现说明](docs/AUTO_CONTROL.md)：完整PID、前进制动介入与释放、倒车滑停及换向许可的软件契约。
- [ESC观测与速度发布](docs/ESC_OBSERVATION.md)：PD15反馈、RC方向状态机、有效位和RC/AUTO边界。
- [接口约束](docs/INTERFACES.md)：串口、控制源与状态语义。
- [车辆数据](docs/VEHICLE.md)：源码参数、硬件数据与标定来源。
- [系统架构图](docs/diagrams/system.svg)：控制源、ESC观测、速度估计和上行状态的关系。
- [AUTO控制图](docs/diagrams/auto-control.svg)、[前进刹车调度图](docs/diagrams/auto-braking.svg)：当前AUTO实现结构。
- [draw.io 图源](docs/diagrams/chassis.drawio)：系统架构图源。

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
