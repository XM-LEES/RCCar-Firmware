# Firmware Acceptance Tools

本目录存放下位机验收工具。工具默认只读源码或只发送 zero/stop 命令；非零运动命令必须显式 armed、限速、限时并记录安全条件。

从 `RCCar-new/` 仓库根目录执行。

| 路径 | 作用 | 是否会让车运动 |
| --- | --- | --- |
| `check_firmware_contract.py` | 静态检查 UART4 Ackermann command、telemetry、状态位定义和串口配置。 | 不会 |

当前检查命令：

```bash
python3 tools/acceptance/check_firmware_contract.py --workspace-root .
python3 tools/acceptance/check_firmware_contract.py --workspace-root . --require-phase1-status-bits
```

第一条命令用于日常契约漂移检查。第二条命令用于阶段 1 最终 PASS 前确认目标状态位符合当前硬件契约。
