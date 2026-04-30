# Module Usage Audit

This audit captures the cleanup state after removing unused source modules from
the current `main` branch.

## Active Runtime Modules

- `SerialControlTask` is created by `MX_FREERTOS_Init()` and consumes
  `g_xQueueROSserial`, which is fed by the UART4 receive callback.
- `ServoBasic_Task` is created by `MX_FREERTOS_Init()` and is the active
  arbitration/output loop for Orin PWM targets, RC passthrough, and guard
  override.
- `servo_rc_capture.c` is active through TIM4 CH1/CH2/CH3 input capture and
  `HAL_TIM_IC_CaptureCallback()`.
- `hall_speed.c` is active through `HallSpeed_Init()` and
  `HAL_GPIO_EXTI_Callback()`.
- `RobotDataTransmitTask` is created by `MX_FREERTOS_Init()` and sends UART4
  base telemetry.
- `show_task` is created by `MX_FREERTOS_Init()` and remains the active OLED
  support task.

## Removed In This Cleanup

| Module | Evidence before cleanup | Cleanup result |
| --- | --- | --- |
| Bluetooth/App control | `BlueToothControlTask` was not created and `g_xQueueBlueTooth` was never allocated. | Deleted Bluetooth/App task files, removed UART2 App receive feed, and removed Keil entries. |
| USB HID gamepad control | `MX_USB_HOST_Init()` was declared but not called, and USB gamepad sources were only project entries. | Deleted `USB_HOST/` and `bsp_gamepad.*`, removed HCD/USB Host project entries and include paths. |
| AutoRecharge | `AutoRechargeTask` was not created and `g_xQueueAutoRecharge` was never allocated. | Deleted task/header, removed charge commands, charge telemetry extension, and charge RGB states. |
| Ranger ultrasonic | Ranger task path was not started while telemetry/RGB still read passive `RangerHAL_*` state. | Deleted Ranger module/header, removed ranger telemetry extension, RGB warning, and `RangerAvoidEN`. |
| ICM20948 IMU | `ImuTask` was still created, and `RobotDataTransmitTask` read `axis_9ValOri` for bytes `8..19` of the ROS frame. | Deleted ICM driver/header/register files and IMU task, removed Keil entries, and fixed ROS bytes `8..19` to zero while preserving the 24-byte frame. |
| RGB APP runtime | `RGBControl_task` was still created, and `SerialControlTask` wrote `userdefine_rgb` on `cmd1=4`. | Deleted `RGBStripControl_task.*`, removed the task from startup/Keil, and kept `cmd1=4` as an ignored compatibility frame. |
| Legacy RC joystick | `rc_joystick.c` was still in Keil, but TIM4 input capture dispatch only calls `ServoRC_IC_CaptureCallback()`. | Deleted `rc_joystick.*`; active RC takeover remains `servo_rc_capture.c`. |
| Legacy `RobotControl_task` compatibility | The task was not created; only reset/log helpers were still used by serial control. | Deleted `RobotControl_task.*` and moved the reset/log parsing helpers into `SerialControl_task.c`. |
| Legacy CAN servo drive | `bsp_ServoDrive.c` depended on `g_xQueueCANopenCallback`, but startup did not allocate that queue and the current APP does not call the driver. | Removed the Keil entry first, then deleted `bsp_ServoDrive.*` from the current cleanup version. Restore from git history or vendor source if needed. |
| `reportErr_task.c` | The task was not created and depended on Bluetooth/App, AutoRecharge, old CAN servo diagnostics, and removed command sources. | Deleted the stale report task and removed its Keil entry. |
| `can_callback.c` | Active cleanup no longer starts the old `WHEELTEC_APP` CAN callback path, and current App code does not call the BSP CAN receive helpers. | Deleted the stale App callback file without modifying the BSP layer. Reintroduce CAN receive handling only from `WHEELTEC_APP` if a future App feature needs it. |
| CAN current target | User confirmed CAN is not used, while `main.c`, IRQ files, HAL config, Keil project files, and `WHEELTEC.ioc` still described CAN1/CAN2 as active peripherals. | Removed CAN startup calls, CAN IRQ handlers, the HAL CAN module switch, Keil `can.c` / `stm32f4xx_hal_can.c` entries, IOC CAN pins/NVIC/IP config, generated `Core/Src/can.c` / `Core/Inc/can.h`, and CAN BSP source from the current cleanup version. Restore from git history or vendor source if needed. |
| USART3/RS485 current target | USART3 initialized PB10/PB11, RX DMA, and IRQ, but the UART callback only restarted receive and no task consumed `rs485_buffer`. | Removed USART3 startup, RX DMA/IRQ, PB10/PB11 USART3 pin config, and IOC USART3 config. |
| USART1 RX path | USART1 TX is useful for debug telemetry, but RX was only restarted and never consumed. | Kept USART1 TX DMA/debug output and removed USART1 RX receive startup, PA10 RX config, and the unused byte buffer. |
| Legacy robot runtime state | `robot_select_init.*` only populated a vendor-style model/geometry cache that no current control path read. | Replaced it with `app_runtime_state.*`, which keeps only voltage, debug level, and UART DMA busy/error counters. Vehicle/Orin defaults now live in `app_vehicle_config.h`. |
| Dormant BSP source | RGBLight, software IIC, EEPROM, UserKey, UserLED, and RTOS debug source remained in the repo but were not part of the current Keil target or APP flow. | Deleted `bsp_RGBLight.*`, `bsp_siic.*`, `bsp_eeprom.*`, `bsp_key.*`, `bsp_led.*`, and `bsp_RTOSdebug.*`; restore from git history or vendor code only with a new APP owner. |
| Dormant Core/IOC hardware setup | TIM9/TIM11 RGB PWM, PB6/PB7 software IIC, UserKey, UserLED, and ENKey were configured but unused by the current product APP. | Removed the unused init calls, GPIO/timer setup, macros, and IOC entries while keeping VersionBit board-version detection. |
| Final current-target residue | TIM4 CH4/PD15, unused RC mode aliases, empty OLED page functions, extra TIM8 CH3/CH4 PWM starts, and old CAN debug naming remained after the aggressive cleanup. | Removed these final edge residues; current RC input is TIM4 CH1/CH2/CH3 and current PWM output is TIM8 CH1/CH2 only. |
| Hall debug telemetry | The 32-byte debug frame was diagnostic-only and conflicted with the fixed ROS UART4 parser contract if mixed into the stream. | Deleted the debug frame path plus IRQ/Callback/accepted-edge counters; kept `g_hall_speed_state` for Keil Watch/OLED diagnosis. |

## Still Present But Not Primary Motion Control

| Module | Current evidence | Follow-up constraint |
| --- | --- | --- |
| `bsp_flash.c` | Still compiled in the current Keil target. Current APP no longer reads the old default-speed or line-diff values at startup. | Keep as the board-supported path for future power-off parameter persistence. |

## Protocol Documentation Note

The ROS UART frame-length contract was not changed by this cleanup. The
manufacturer protocol table and the firmware's IMU/Z-axis semantics already
diverged in the initial repository commit `6dd55e9`; commit `76fc7c2` made the
Z-axis angular-rate interpretation more explicit through `vz_rad_s` naming. The
current protocol documentation therefore keeps the vendor table as historical
reference and treats the running firmware's 24-byte uplink / 11-byte downlink as
the project contract.

## Follow-up Order

1. Rebuild with Keil to confirm the pruned project file compiles on the target
   toolchain.
2. Reintroduce deleted BSP helpers only from git history or vendor code, and
   only together with an APP-level owner that calls them.
3. Recheck `WHEELTEC.ioc` before any future CubeMX regeneration; it is now
   aligned with this cleanup state and no longer configures USB Host,
   USB OTG FS, Bluetooth/App USART2, Ranger TIM2/TIM3, Ranger GPIO, CAN1/2,
   USART3/RS485, TIM9/TIM11, TIM4 CH4/PD15, RGB PWM, software IIC, UserKey,
   UserLED, or ENKey.
