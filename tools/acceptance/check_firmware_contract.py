#!/usr/bin/env python3
"""Check the STM32 firmware contract used by phase-1 acceptance."""

from __future__ import annotations

import argparse
import re
import sys
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class Check:
    name: str
    passed: bool
    detail: str


def read_text(root: Path, relative: str) -> str:
    path = root / relative
    if not path.exists():
        raise FileNotFoundError(f"missing {relative}")
    return path.read_text(encoding="utf-8", errors="replace")


def contains(text: str, needle: str) -> bool:
    return needle in text


def matches(text: str, pattern: str) -> bool:
    return re.search(pattern, text, flags=re.MULTILINE | re.DOTALL) is not None


def add(results: list[Check], name: str, condition: bool, detail: str) -> None:
    results.append(Check(name=name, passed=condition, detail=detail if condition else f"missing: {detail}"))


def check_command_parser(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/SerialControl_task.c")
    results: list[Check] = []
    add(results, "command_frame_len", contains(text, "#define ROS_CMD_FRAME_LEN 11U"), "ROS_CMD_FRAME_LEN 11U")
    add(results, "command_id", contains(text, "#define ROS_CMD_ACKERMANN 0x01U"), "ROS_CMD_ACKERMANN 0x01U")
    add(results, "command_flags", all(needle in text for needle in [
        "ROS_CMD_FLAG_ENABLE",
        "ROS_CMD_FLAG_BRAKE",
        "ROS_CMD_FLAG_CLEAR_FAULT",
        "ROS_CMD_FLAG_EMERGENCY_STOP",
    ]), "enable/brake/clear_fault/emergency_stop flags")
    add(results, "command_head_tail", contains(text, "recv == 0x7BU") and contains(text, "!= 0x7DU"), "0x7B head and 0x7D tail")
    add(results, "command_bcc", contains(text, "Calculate_BCC(roscmdBuf, cmdLen - 2U)"), "command BCC over len-2")
    add(results, "command_fields", all(needle in text for needle in [
        "serial_control_read_i16_be(&roscmdBuf[3])",
        "serial_control_read_i16_be(&roscmdBuf[5])",
        "ServoBasic_UpdateAckermannFromOrin",
    ]), "speed_mmps, steering_mrad, Ackermann actuator call")
    add(results, "rc_override_guard", all(needle in text for needle in [
        "ServoBasic_IsRcOverrideActive",
        "serial_control_send_zero_command",
        "allow_serial_motion",
    ]), "RC override blocks non-zero serial motion")
    add(results, "clear_fault_guard", contains(text, "serial_control_try_clear_diagnostics"), "guarded CLEAR_FAULT path")
    return results


def check_telemetry(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/data_task.c")
    results: list[Check] = []
    add(results, "telemetry_frame_len", contains(text, "#define BaseFRAME_LEN  24U"), "BaseFRAME_LEN 24U")
    add(results, "telemetry_head_tail", contains(text, "#define BaseFRAME_HEAD 0x7B") and contains(text, "#define BaseFRAME_TAIL 0x7D"), "0x7B head and 0x7D tail")
    add(results, "telemetry_bcc", contains(text, "basebuffer[22] = Calculate_BCC(basebuffer, 22U)"), "telemetry BCC at byte 22 over first 22 bytes")
    add(results, "telemetry_transport", contains(text, "static UART_HandleTypeDef *serial = &huart4") and contains(text, "HAL_UART_Transmit_DMA(serial, basebuffer, BaseFRAME_LEN)"), "UART4 DMA telemetry transport")

    status_defs = [
        "STATUS_BIT_FAULT_LATCHED",
        "STATUS_BIT_COMMAND_TIMEOUT",
        "STATUS_BIT_RC_OVERRIDE_ACTIVE",
        "STATUS_BIT_ESTOP_ACTIVE",
        "STATUS_BIT_BRAKE_ACTIVE",
        "STATUS_BIT_AUTO_ENABLED",
        "STATUS_BIT_HALL_FEEDBACK_VALID",
        "STATUS_BIT_HALL_FAULT",
        "STATUS_BIT_STEERING_FEEDBACK_VALID",
        "STATUS_BIT_STEERING_IS_MEASURED",
        "STATUS_BIT_STEERING_FAULT",
        "STATUS_BIT_BATTERY_VALID",
        "STATUS_BIT_BATTERY_LOW",
        "STATUS_BIT_BATTERY_CRITICAL",
        "STATUS_BIT_SPEED_SATURATED",
        "STATUS_BIT_STEERING_SATURATED",
        "STATUS_BIT_ACCEL_LIMITED",
        "STATUS_BIT_STEERING_RATE_LIMITED",
        "STATUS_BIT_FRAME_ERROR_SEEN",
    ]
    add(results, "status_bit_definitions", all(needle in text for needle in status_defs), "complete status bit definitions")

    status_assignments = [
        "STATUS_BIT_COMMAND_TIMEOUT",
        "STATUS_BIT_RC_OVERRIDE_ACTIVE",
        "STATUS_BIT_ESTOP_ACTIVE",
        "STATUS_BIT_BRAKE_ACTIVE",
        "STATUS_BIT_AUTO_ENABLED",
        "STATUS_BIT_HALL_FEEDBACK_VALID",
        "STATUS_BIT_HALL_FAULT",
        "STATUS_BIT_STEERING_FEEDBACK_VALID",
        "STATUS_BIT_BATTERY_VALID",
        "STATUS_BIT_FRAME_ERROR_SEEN",
    ]
    add(results, "current_status_assignments", all(matches(text, rf"status_bits\s*\|=\s*{needle}") for needle in status_assignments), "current implemented status bit assignments")

    layout_needles = [
        "basebuffer[1] = status_flags",
        "basebuffer[2] = seq++",
        "write_i32_be(&basebuffer[3], hall_delta_count)",
        "write_i16_be(&basebuffer[7], clamp_float_to_i16(speed_mps * 1000.0f))",
        "write_i16_be(&basebuffer[9], clamp_float_to_i16(steering_angle_rad * 1000.0f))",
        "write_i16_be(&basebuffer[11], clamp_float_to_i16(yaw_rate_rad_s * 1000.0f))",
        "write_u16_be(&basebuffer[15], (dt_ms > 65535U) ? 65535U : (uint16_t)dt_ms)",
        "write_u32_be(&basebuffer[17], status_bits)",
        "basebuffer[23] = BaseFRAME_TAIL",
    ]
    battery_slot_ok = (
        "write_u16_be(&basebuffer[13], battery_mv)" in text
        or "write_u16_be(&basebuffer[13], clamp_float_to_u16(g_app_runtime_state.voltage_v * 1000.0f))" in text
    )
    add(results, "telemetry_layout", all(needle in text for needle in layout_needles) and battery_slot_ok, "24-byte telemetry layout")
    return results


def check_uart(root: Path) -> list[Check]:
    text = read_text(root, "Core/Src/usart.c")
    results: list[Check] = []
    add(results, "uart4_instance", contains(text, "huart4.Instance = UART4"), "UART4 instance")
    add(results, "uart4_115200_8n1", all(needle in text for needle in [
        "huart4.Init.BaudRate = 115200",
        "huart4.Init.WordLength = UART_WORDLENGTH_8B",
        "huart4.Init.StopBits = UART_STOPBITS_1",
        "huart4.Init.Parity = UART_PARITY_NONE",
    ]), "UART4 115200 8N1")
    add(results, "uart4_tx_rx", contains(text, "huart4.Init.Mode = UART_MODE_TX_RX"), "UART4 TX/RX mode")
    return results


def check_phase1_target_status_bits(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/data_task.c")
    required_assignments = [
        "STATUS_BIT_FAULT_LATCHED",
        "STATUS_BIT_STEERING_FAULT",
        "STATUS_BIT_BATTERY_LOW",
        "STATUS_BIT_BATTERY_CRITICAL",
        "STATUS_BIT_SPEED_SATURATED",
        "STATUS_BIT_STEERING_SATURATED",
        "STATUS_BIT_ACCEL_LIMITED",
        "STATUS_BIT_STEERING_RATE_LIMITED",
    ]
    results = [
        Check(
            name=f"complete_assignment_{name}",
            passed=matches(text, rf"status_bits\s*\|=\s*{name}"),
            detail=f"{name} assignment",
        )
        for name in required_assignments
    ]
    results.append(
        Check(
            name="steering_is_measured_current_hardware_false",
            passed=not matches(text, r"status_bits\s*\|=\s*STATUS_BIT_STEERING_IS_MEASURED"),
            detail="STATUS_BIT_STEERING_IS_MEASURED remains false without measured steering hardware",
        )
    )
    return results


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--workspace-root",
        type=Path,
        default=Path(__file__).resolve().parents[2],
        help="RCCar-new repository root",
    )
    parser.add_argument(
        "--require-phase1-status-bits",
        action="store_true",
        help="fail until phase-1 target status bits match the current hardware contract",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.workspace_root.resolve()
    results = check_command_parser(root) + check_telemetry(root) + check_uart(root)
    if args.require_phase1_status_bits:
        results += check_phase1_target_status_bits(root)

    for result in results:
        status = "PASS" if result.passed else "FAIL"
        print(f"{status} {result.name}: {result.detail}")

    if not args.require_phase1_status_bits:
        print("INFO phase-1 target status bits are checked with --require-phase1-status-bits")

    return 0 if all(result.passed for result in results) else 1


if __name__ == "__main__":
    sys.exit(main())
