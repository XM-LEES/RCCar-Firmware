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
        "ROS_CMD_FLAG_SOFTWARE_STOP",
        "ROS_CMD_FLAG_ALLOWED_MASK",
    ]), "enable/brake/clear_fault/software_stop flags and allowed mask")
    add(results, "command_head_tail", contains(text, "recv != 0x7BU") and contains(text, "!= 0x7DU"), "0x7B head and 0x7D tail")
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
    add(results, "rc_override_zero_refresh", all(needle in text for needle in [
        "if (allow_serial_motion == 0U)",
        "serial_control_send_zero_command()",
        "ServoBasic_UpdateAckermannFromOrin(0.0f, 0.0f, 1U, 0U, 0U)",
        "command timeout is 250 ms",
    ]), "blocked non-zero serial commands refresh only an explicit zero target during RC override")
    add(results, "clear_fault_guard", all(needle in text for needle in [
        "serial_control_try_clear_diagnostics",
        "speed_mmps != 0 || steering_mrad != 0",
    ]), "CLEAR_FAULT requires a zero-motion frame")
    add(results, "clear_fault_request_handoff", contains(
        text, "AppRuntime_RequestFaultClear()"
    ) and all(needle not in text for needle in [
        "g_app_runtime_state.uart4_rx_frame_error_seen = 0U",
        "AppRuntime_TryClearFaultLatch()",
    ]), "UART parser requests diagnostic clearing without racing telemetry-owned state")
    add(results, "binary_only_uart4", all(needle not in text for needle in [
        "serial_control_check_reset",
        "serial_control_set_debug_level",
        "NVIC_SystemReset",
        "bsp_buzzer.h",
    ]), "UART4 parser has no reset or LOG side channel")
    add(results, "command_resynchronization", all(needle in text for needle in [
        "serial_control_retain_next_header",
        "roscmdCount = serial_control_retain_next_header(roscmdBuf, cmdLen)",
    ]), "invalid frame retains the next possible 0x7B header")
    add(results, "command_reserved_zero", all(needle in text for needle in [
        "roscmdBuf[7] != 0U",
        "roscmdBuf[8] != 0U",
    ]), "command reserved bytes must remain zero")
    add(results, "command_unknown_flags_rejected", contains(
        text, "flags & (uint8_t)(~ROS_CMD_FLAG_ALLOWED_MASK)"
    ), "undefined command flag bits are rejected")
    return results


def check_telemetry(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/data_task.c")
    vehicle_config_text = read_text(root, "WHEELTEC_APP/Inc/app_vehicle_config.h")
    runtime_state_text = read_text(root, "WHEELTEC_APP/Inc/app_runtime_state.h")
    results: list[Check] = []
    add(results, "telemetry_frame_len", contains(text, "#define BaseFRAME_LEN  24U"), "BaseFRAME_LEN 24U")
    add(results, "telemetry_head_tail", contains(text, "#define BaseFRAME_HEAD 0x7B") and contains(text, "#define BaseFRAME_TAIL 0x7D"), "0x7B head and 0x7D tail")
    add(results, "telemetry_bcc", contains(text, "basebuffer[22] = Calculate_BCC(basebuffer, 22U)"), "telemetry BCC at byte 22 over first 22 bytes")
    add(results, "telemetry_transport", contains(text, "static UART_HandleTypeDef *serial = &huart4") and contains(text, "HAL_UART_Transmit_DMA(serial, basebuffer, BaseFRAME_LEN)"), "UART4 DMA telemetry transport")
    add(results, "telemetry_protocol_id", all(needle in text for needle in [
        "#define TELEMETRY_PROTOCOL_ID 0xA1U",
        "basebuffer[21] = TELEMETRY_PROTOCOL_ID",
    ]), "byte 21 carries Ackermann telemetry protocol id 0xA1")

    status_defs = [
        "STATUS_BIT_FAULT_LATCHED",
        "STATUS_BIT_COMMAND_TIMEOUT",
        "STATUS_BIT_RC_OVERRIDE_ACTIVE",
        "STATUS_BIT_STOP_OVERRIDE_ACTIVE",
        "STATUS_BIT_BRAKE_ACTIVE",
        "STATUS_BIT_AUTO_ENABLED",
        "STATUS_BIT_HALL_FEEDBACK_VALID",
        "STATUS_BIT_HALL_FAULT",
        "STATUS_BIT_STEERING_ESTIMATE_VALID",
        "STATUS_BIT_STEERING_IS_MEASURED",
        "STATUS_BIT_RC_INPUT_FAULT",
        "STATUS_BIT_BATTERY_VALID",
        "STATUS_BIT_HALL_STANDSTILL_CONFIRMED",
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
        "STATUS_BIT_STOP_OVERRIDE_ACTIVE",
        "STATUS_BIT_BRAKE_ACTIVE",
        "STATUS_BIT_AUTO_ENABLED",
        "STATUS_BIT_HALL_FEEDBACK_VALID",
        "STATUS_BIT_HALL_STANDSTILL_CONFIRMED",
        "STATUS_BIT_HALL_FAULT",
        "STATUS_BIT_STEERING_ESTIMATE_VALID",
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
    add(results, "battery_raw_telemetry_only", all(needle not in text for needle in [
        "STATUS_BIT_BATTERY_LOW",
        "STATUS_BIT_BATTERY_CRITICAL",
        "battery_mv_is_low",
        "battery_mv_is_critical",
        "APP_FAULT_SOURCE_BATTERY_LOW",
        "APP_FAULT_SOURCE_BATTERY_CRITICAL",
    ]) and all(needle not in vehicle_config_text for needle in [
        "APP_BATTERY_LOW_MV",
        "APP_BATTERY_CRITICAL_MV",
    ]) and all(needle not in runtime_state_text for needle in [
        "APP_FAULT_SOURCE_BATTERY_LOW",
        "APP_FAULT_SOURCE_BATTERY_CRITICAL",
    ]), "battery is raw mV telemetry only, without firmware low-voltage thresholds or faults")
    return results


def check_hall_direction_sources(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    hall_text = read_text(root, "WHEELTEC_APP/hall_speed.c")
    hall_header_text = read_text(root, "WHEELTEC_APP/Inc/hall_speed.h")
    vehicle_config_text = read_text(root, "WHEELTEC_APP/Inc/app_vehicle_config.h")
    data_text = read_text(root, "WHEELTEC_APP/data_task.c")
    results: list[Check] = []
    add(results, "auto_hall_direction_source", all(needle in text for needle in [
        "command_direction = get_vx_direction(limited_speed_mps)",
        "HallSpeed_SetCommandDirection(command_direction)",
    ]), "automatic Ackermann speed sets Hall direction")
    add(results, "rc_hall_direction_source", all(needle in text for needle in [
        "static int8_t get_rc_throttle_direction(void)",
        "g_rc_throttle_current",
        "center_us + neutral_hold_us",
        "HallSpeed_SetCommandDirection(get_rc_throttle_direction())",
    ]), "RC passthrough throttle sets Hall direction")
    add(
        results,
        "unknown_direction_zero_delta_gate",
        contains(data_text, "else if (snapshot->direction == 0)") and contains(data_text, "delta = 0"),
        "unknown Hall direction keeps telemetry delta zero",
    )
    add(results, "hall_geometry", contains(hall_text, "#define HALL_WHEEL_DIAMETER_M            0.230f"), "Hall wheel diameter is 0.230 m")
    add(results, "hall_dwt_rollover_safe", all(needle in hall_header_text for needle in [
        "uint32_t last_event_cycles",
        "uint32_t last_raw_event_cycles",
        "uint32_t zero_command_since_cycles",
    ]) and all(needle in hall_text for needle in [
        "const uint32_t elapsed_cycles = now_cycles - start_cycles",
        "return elapsed_cycles / cycles_per_us",
        "g_hall_speed_started_cycles",
    ]) and "DWT_CYCCNT / cycles_per_us" not in hall_text,
        "raw DWT cycles are subtracted before conversion so the 32-bit rollover is safe")
    add(results, "hall_glitch_burst_confirmation", all(needle in hall_header_text for needle in [
        "uint32_t last_raw_event_cycles",
        "uint8_t raw_event_origin_valid",
        "uint8_t consecutive_short_event_count",
    ]) and contains(
        vehicle_config_text, "#define APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS         3U"
    ) and matches(
        hall_text,
        r"if\s*\(raw_elapsed_us\s*<\s*HALL_MIN_EVENT_INTERVAL_US\)\s*\{"
        r".*?consecutive_short_event_count\s*<"
        r".*?APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS"
        r".*?consecutive_short_event_count\+\+;"
        r".*?consecutive_short_event_count\s*=="
        r".*?APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS"
        r".*?fault_count\+\+;"
        r".*?return;"
        r".*?consecutive_short_event_count\s*=\s*0U;",
    ), "one or two impossible raw intervals are rejected, while three consecutive intervals latch a Hall fault")
    add(results, "unknown_direction_invalid_speed", contains(hall_text, "snapshot.direction == 0"), "unknown Hall direction cannot produce signed speed")
    add(results, "signed_hall_status_requires_direction", all(needle in data_text for needle in [
        "signed_speed_valid = ServoBasic_GetAckermannFeedback",
        "if (signed_speed_valid != 0U)",
        "status_bits |= STATUS_BIT_HALL_FEEDBACK_VALID",
    ]) and contains(hall_text, "snapshot.direction == 0"), "signed Hall feedback is valid only when both magnitude and command-derived direction are known")
    add(results, "hall_direction_retained_during_coast", all(needle in hall_header_text for needle in [
        "int8_t direction",
        "int8_t command_direction",
        "uint32_t zero_command_since_cycles",
        "uint8_t stationary_confirmed",
    ]) and all(needle in hall_text for needle in [
        "g_hall_speed_state.command_direction = command_direction",
        "g_hall_speed_state.zero_command_since_cycles = now_cycles",
        "A zero request deliberately retains direction",
        "snapshot.direction = 0",
    ]), "zero command retains the last sign until Hall silence confirms standstill")
    add(results, "hall_timeout_requires_fresh_period", all(needle in hall_text for needle in [
        "g_hall_speed_state.timeout_active == 0U",
        "if (period_accepted == 0U)",
        "A first edge after acquisition or timeout only establishes an origin.",
        "A timed-out measurement stays unavailable until a fresh Hall edge.",
    ]), "timed-out Hall feedback cannot revive a stale period across a later counter rollover")
    add(results, "hall_standstill_requires_zero_command", matches(
        hall_text,
        r"if\s*\(zero_command_quiet\s*!=\s*0U\)\s*\{"
        r".*?snapshot\.direction\s*=\s*0;"
        r".*?snapshot\.stationary_confirmed\s*=\s*1U;",
    ) and contains(
        hall_text, "No pulses under a non-zero request is unknown, not zero."
    ), "Hall silence is publishable as standstill only without a current motion request")
    add(results, "hall_motion_and_standstill_status_exclusive", matches(
        data_text,
        r"if\s*\(signed_speed_valid\s*!=\s*0U\)\s*\{"
        r".*?STATUS_BIT_HALL_FEEDBACK_VALID;"
        r".*?\}\s*else\s*\{"
        r".*?stationary_confirmed\s*!=\s*0U"
        r".*?STATUS_BIT_HALL_STANDSTILL_CONFIRMED;",
    ), "telemetry cannot mark measured motion and Hall-confirmed standstill together")
    return results


def check_vehicle_defaults(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/Inc/app_vehicle_config.h")
    control_text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    results: list[Check] = []
    expected = [
        "#define APP_ORIN_PWM_TIMEOUT_DEFAULT_MS           250U",
        "#define APP_ORIN_ACKERMANN_WHEELBASE_MM           600U",
        "#define APP_ORIN_ACKERMANN_TRACK_WIDTH_MM         500U",
        "#define APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM        115U",
        "#define APP_ORIN_ACKERMANN_MAX_STEERING_MRAD      349U",
        "#define APP_ORIN_MIN_COMMAND_SPEED_MMPS            300U",
        "#define APP_ORIN_VX_FORWARD_CAP_MMPS            10000U",
        "#define APP_ORIN_VX_REVERSE_CAP_MMPS             3000U",
        "#define APP_ORIN_VX_MAX_DEFAULT_MMPS            10000U",
        "#define APP_HALL_SPEED_LIMIT_MMPS               12000U",
        "#define APP_HALL_SPEED_LIMIT_RELEASE_MMPS       10500U",
        "#define APP_HALL_SPEED_LIMIT_CONFIRM_SAMPLES        3U",
        "#define APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS         3U",
        "#define APP_ORIN_ACCEL_LIMIT_MMPS2               4000U",
        "#define APP_ORIN_SERVO_CENTER_US                 1500U",
        "#define APP_ORIN_SERVO_RANGE_US                   395U",
        "#define APP_ORIN_STEERING_PWM_DIRECTION_SIGN        (+1)",
        "#define APP_RC_GUARD_ENABLE_DEFAULT                 0U",
    ]
    add(results, "vehicle_defaults", all(needle in text for needle in expected), "confirmed geometry, speed bounds, 0.3 m/s minimum, servo calibration, timeout, and disabled unverified guard")
    add(
        results,
        "forward_speed_envelope",
        "{10000U, APP_ORIN_ESC_FORWARD_MAX_US}" in control_text,
        "10 m/s forward endpoint reaches the existing configured ESC limit",
    )
    add(
        results,
        "hall_overspeed_confirmation",
        all(needle in control_text for needle in [
            "s_hall_speed_limit_over_count",
            "s_hall_speed_limit_over_count++",
            "s_hall_speed_limit_over_count >= confirm_samples",
        ]),
        "Hall overspeed requires consecutive confirmations before neutral output",
    )
    add(
        results,
        "orin_steering_direction_and_feedback",
        all(needle in control_text for needle in [
            "(steering_angle_rad / max_steering_rad) *",
            "(float)APP_ORIN_STEERING_PWM_DIRECTION_SIGN",
            "return ratio * (float)APP_ORIN_STEERING_PWM_DIRECTION_SIGN *",
        ]),
        "Orin steering PWM follows the field-observed chassis direction and telemetry preserves the standard logical sign",
    )
    return results


def check_control_output_fallbacks(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    results: list[Check] = []
    add(results, "no_zero_or_minimum_stop_pwm", all(needle not in text for needle in [
        "apply_esc_pulse(0U)",
        "apply_servo_pulse(0U)",
        "apply_esc_pulse(ESC_PWM_MIN_PULSE_US)",
    ]), "stop and timeout paths do not emit PWM 0 or ESC minimum")
    add(results, "neutral_stop_and_timeout", text.count("apply_esc_pulse(get_orin_esc_center_pulse())") >= 3 and text.count("apply_servo_pulse(get_orin_servo_center_pulse())") >= 3, "guard, software stop, and timeout paths use configured centers")
    add(results, "candidate_rc_arbitration_preserved", all(needle in text for needle in [
        "const uint8_t manual_override = rc_manual_override_requested()",
        "const uint8_t serial_active = orin_pwm_is_active()",
        "const uint8_t rc_available = rc_passthrough_is_available()",
        "if (serial_active == 0U)",
        "get_rc_override_release_hold_ms()",
    ]), "feature/ackermann-chassis RC arbitration remains present")
    add(results, "rc_release_source_distinguished", all(needle in text for needle in [
        "g_rc_override_release_hold_required",
        "g_rc_override_release_hold_required == 0U && centered != 0U",
        "set_rc_override_state(1U, 0U, 1U)",
        "set_rc_override_state(1U, 0U, 0U)",
    ]), "idle RC passthrough releases immediately, but a real manual override keeps the 500 ms hold")
    return results


def check_fault_recovery(root: Path) -> list[Check]:
    servo_text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    serial_text = read_text(root, "WHEELTEC_APP/SerialControl_task.c")
    data_text = read_text(root, "WHEELTEC_APP/data_task.c")
    hall_text = read_text(root, "WHEELTEC_APP/hall_speed.c")
    hall_header_text = read_text(root, "WHEELTEC_APP/Inc/hall_speed.h")
    runtime_text = read_text(root, "WHEELTEC_APP/app_runtime_state.c")
    runtime_header_text = read_text(root, "WHEELTEC_APP/Inc/app_runtime_state.h")
    results: list[Check] = []

    add(results, "fault_clear_request_counter", all(needle in runtime_header_text for needle in [
        "volatile uint8_t uart4_rx_frame_error_seen",
        "volatile uint32_t fault_clear_request_count",
        "void AppRuntime_RequestFaultClear(void)",
        "uint32_t AppRuntime_GetFaultClearRequestCount(void)",
    ]) and all(needle in runtime_text for needle in [
        "g_app_runtime_state.fault_clear_request_count++",
        "return g_app_runtime_state.fault_clear_request_count",
    ]) and contains(serial_text, "AppRuntime_RequestFaultClear()"),
        "cross-task fault-clear request sequence counter")

    add(results, "hall_fault_diagnostic_clear", contains(
        hall_header_text, "void HallSpeed_ClearFaultCount(void)"
    ) and matches(
        hall_text,
        r"void\s+HallSpeed_ClearFaultCount\s*\(void\)\s*\{"
        r".*?__disable_irq\(\).*?g_hall_speed_state\.fault_count\s*=\s*0U"
        r".*?g_hall_speed_state\.consecutive_short_event_count\s*=\s*0U"
        r".*?__enable_irq\(\).*?\}",
    ), "Hall fault history and pending glitch streak have an interrupt-safe explicit clear")

    add(results, "telemetry_owned_fault_clear", matches(
        data_text,
        r"if\s*\(clear_request_count\s*!=\s*handled_clear_request_count\)\s*\{"
        r".*?HallSpeed_ClearFaultCount\(\);"
        r".*?uart4_rx_frame_error_seen\s*=\s*0U;"
        r".*?clear_fault_requested\s*=\s*1U;"
        r".*?HallSpeed_GetState\(\)"
        r".*?AppRuntime_UpdateFaultSources\(active_fault_sources\);"
        r".*?if\s*\(clear_fault_requested\s*!=\s*0U\)\s*\{"
        r".*?AppRuntime_TryClearFaultLatch\(\);",
    ), "telemetry task clears historical sources, recomputes live sources, then clears the aggregate latch")

    fault_branch = servo_text.find("if (fault_active != 0U)")
    absent_branch = servo_text.find("if (raw_present == 0U || raw == 0U)")
    add(results, "rc_capture_fault_survives_signal_timeout",
        fault_branch >= 0 and absent_branch > fault_branch,
        "capture fault tracking is evaluated before clean signal absence")
    add(results, "rc_fault_freeze_window", all(needle in servo_text for needle in [
        "rc_channel_fault_is_persistent",
        "(now_ms - state->invalid_since_ms) >= get_rc_glitch_freeze_ms()",
        "throttle_fault_persistent",
        "steering_fault_persistent",
    ]), "RC capture faults become live only after the configured glitch-freeze interval")
    add(results, "disabled_guard_not_fault_source", contains(
        servo_text, "(g_rc_guard_enable != 0U && guard_fault != 0U)"
    ), "disabled unverified guard input cannot assert RC_INPUT_FAULT")
    add(results, "transient_rc_glitch_not_reported_live", contains(
        servo_text, "diagnostics.steering_fault = (g_rc_input_fault_active != 0U) ? 1U : 0U;"
    ), "diagnostics report the persistent RC input fault, not the immediate glitch watch value")
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
        "STATUS_BIT_RC_INPUT_FAULT",
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
    results = (
        check_command_parser(root)
        + check_telemetry(root)
        + check_hall_direction_sources(root)
        + check_vehicle_defaults(root)
        + check_control_output_fallbacks(root)
        + check_fault_recovery(root)
        + check_uart(root)
    )
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
