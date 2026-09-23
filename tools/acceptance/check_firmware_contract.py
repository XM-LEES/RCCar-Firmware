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
    ]), "byte 21 carries Hall-speed telemetry protocol id 0xA1")

    status_defs = [
        "STATUS_BIT_FAULT_LATCHED",
        "STATUS_BIT_COMMAND_TIMEOUT",
        "STATUS_BIT_RC_OVERRIDE_ACTIVE",
        "STATUS_BIT_STOP_OVERRIDE_ACTIVE",
        "STATUS_BIT_BRAKE_ACTIVE",
        "STATUS_BIT_AUTO_ENABLED",
        "STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID",
        "STATUS_BIT_HALL_FAULT",
        "STATUS_BIT_STEERING_ESTIMATE_VALID",
        "STATUS_BIT_STEERING_IS_MEASURED",
        "STATUS_BIT_RC_INPUT_FAULT",
        "STATUS_BIT_BATTERY_VALID",
        "STATUS_BIT_HALL_STANDSTILL_CONFIRMED",
        "STATUS_BIT_HALL_SPEED_VALID",
        "STATUS_BIT_SPEED_SATURATED",
        "STATUS_BIT_STEERING_SATURATED",
        "STATUS_BIT_ACCEL_LIMITED",
        "STATUS_BIT_STEERING_RATE_LIMITED",
        "STATUS_BIT_FRAME_ERROR_SEEN",
        "STATUS_BIT_HALL_STOP_CONFIRMED",
        "STATUS_BIT_ESC_FE32_FRESH",
        "STATUS_BIT_ESC_RPM_RAW_VALID",
        "STATUS_BIT_ESC_SPEED_CALIBRATION_VALID",
        "STATUS_BIT_VEHICLE_DIRECTION_KNOWN",
        "STATUS_BIT_ESC_SOFT_UART_RX_ERROR",
        "STATUS_BIT_AUTO_PROPULSION_AUTHORIZED",
        "STATUS_BIT_CLOSED_LOOP_ACTIVE",
        "STATUS_BIT_TRACKING_BRAKE_ACTIVE",
        "STATUS_BIT_MODE2_OPPOSITE_ARMED",
        "STATUS_BIT_MODE2_STATE_AMBIGUOUS",
        "STATUS_BIT_MODE2_CONTROL_INHIBITED",
        "STATUS_BIT_MODE2_CONFIG_VALID",
    ]
    add(results, "status_bit_definitions", all(needle in text for needle in status_defs), "complete status bit definitions")

    status_assignments = [
        "STATUS_BIT_COMMAND_TIMEOUT",
        "STATUS_BIT_RC_OVERRIDE_ACTIVE",
        "STATUS_BIT_STOP_OVERRIDE_ACTIVE",
        "STATUS_BIT_BRAKE_ACTIVE",
        "STATUS_BIT_AUTO_ENABLED",
        "STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID",
        "STATUS_BIT_HALL_STANDSTILL_CONFIRMED",
        "STATUS_BIT_HALL_SPEED_VALID",
        "STATUS_BIT_HALL_FAULT",
        "STATUS_BIT_STEERING_ESTIMATE_VALID",
        "STATUS_BIT_BATTERY_VALID",
        "STATUS_BIT_FRAME_ERROR_SEEN",
        "STATUS_BIT_HALL_STOP_CONFIRMED",
        "STATUS_BIT_ESC_FE32_FRESH",
        "STATUS_BIT_ESC_RPM_RAW_VALID",
        "STATUS_BIT_ESC_SPEED_CALIBRATION_VALID",
        "STATUS_BIT_VEHICLE_DIRECTION_KNOWN",
        "STATUS_BIT_ESC_SOFT_UART_RX_ERROR",
        "STATUS_BIT_AUTO_PROPULSION_AUTHORIZED",
        "STATUS_BIT_CLOSED_LOOP_ACTIVE",
        "STATUS_BIT_TRACKING_BRAKE_ACTIVE",
        "STATUS_BIT_MODE2_OPPOSITE_ARMED",
        "STATUS_BIT_MODE2_STATE_AMBIGUOUS",
        "STATUS_BIT_MODE2_CONTROL_INHIBITED",
        "STATUS_BIT_MODE2_CONFIG_VALID",
    ]
    add(results, "current_status_assignments", all(matches(text, rf"status_bits\s*\|=\s*{needle}") for needle in status_assignments), "current implemented status bit assignments")

    layout_needles = [
        "basebuffer[1] = status_flags",
        "basebuffer[2] = seq++",
        "write_i32_be(&basebuffer[3], (int32_t)(hall_speed_mps * 1000.0f))",
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
    add(results, "telemetry_esc_action_flags", all(needle in text for needle in [
        "#define TELEMETRY_FLAG_ESC_ACTION_SHIFT         6U",
        "#define TELEMETRY_FLAG_ESC_ACTION_MASK       0xC0U",
        "servo_diagnostics.esc_action <<",
        "TELEMETRY_FLAG_ESC_ACTION_SHIFT",
    ]) and "TELEMETRY_FLAG_STEERING_IS_MEASURED" not in text,
        "byte 1 bits 7:6 encode UNKNOWN/NEUTRAL/DRIVE/BRAKE without changing the 24-byte layout")
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


def check_speed_feedback_sources(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    longitudinal_text = read_text(root, "WHEELTEC_APP/longitudinal_controller.c")
    mode2_text = read_text(root, "WHEELTEC_APP/mode2_drive_gate.c")
    hall_text = read_text(root, "WHEELTEC_APP/hall_speed.c")
    hall_header_text = read_text(root, "WHEELTEC_APP/Inc/hall_speed.h")
    vehicle_config_text = read_text(root, "WHEELTEC_APP/Inc/app_vehicle_config.h")
    data_text = read_text(root, "WHEELTEC_APP/data_task.c")
    show_text = read_text(root, "WHEELTEC_APP/show_task.c")
    esc_telemetry_text = read_text(root, "WHEELTEC_APP/esc_telemetry.c")
    rc_direction_text = read_text(root, "WHEELTEC_APP/rc_direction_observer.c")
    results: list[Check] = []
    add(results, "auto_uses_esc_motion_not_hall_speed", all(needle in text for needle in [
        "servo_basic_update_esc_feedback",
        "EscTelemetry_PendingSamples()",
        "EscTelemetry_PopSample(&item)",
        "EscMotionEstimator_ObserveSample(&s_esc_motion_estimator",
        "servo_basic_observe_auto_sample(&item, now_ms)",
        "servo_basic_feed_auto_pid()",
        "LongitudinalController_ObserveFeedback(&s_longitudinal_controller",
        "Mode2DriveGate_Observe(&s_mode2_drive_gate",
        "Mode2DriveGate_EvaluateControl(&s_mode2_drive_gate",
        "s_esc_motion_estimate.speed_magnitude_mps",
    ]) and all(needle not in text for needle in [
        "HallSpeed_GetSignedSpeedMps",
        "HallSpeed_SetCommandDirection(command_direction)",
    ]), "automatic Ackermann speed, PID, and gate consume ESC samples rather than Hall speed")
    add(results, "rc_direction_observer_unifies_hall_and_esc", all(needle in text for needle in [
        '#include "rc_direction_observer.h"',
        "servo_basic_update_rc_direction_observer(now_ms,",
        "RcDirectionObserver_Update(",
        "input.state_raw",
        "input.moving_evidence",
        "input.applied_pwm_us = pwm_us",
        "EscTelemetry_ContextPwm(item.output_context)",
        "return (s_rc_direction_result.direction_known != 0U) ?",
        "(int8_t)s_rc_direction_result.direction : 0",
        "s_rc_direction_result.direction_known != 0U",
        "s_esc_stop_confirmed == 0U",
    ]) and all(needle in rc_direction_text for needle in [
        "RC_DIRECTION_OBSERVER_STATE_NEUTRAL",
        "RC_DIRECTION_OBSERVER_STATE_DRIVE",
        "RC_DIRECTION_OBSERVER_STATE_BRAKE",
        "observer->saw_non_drive_since_confirmed",
        "observer->pending_count >= 2U",
    ]) and all(needle not in text for needle in [
        "RC_HALL_MODE2_BRAKING",
        "RC_HALL_MODE2_BRAKE_STOPPED",
        "RC_HALL_MODE2_OPPOSITE_ARMED",
        "rc_hall_mode2_update()",
        "s_vehicle_direction = (int8_t)s_rc_direction_result.direction",
    ]), "RC Hall and ESC signs share the FE32 action/PWM-side observer without the old symmetric command-history state machine")
    add(results, "auto_hall_direction_uses_confirmed_observer_direction", all(needle in text for needle in [
        "RcDirectionObserver_Update(&s_auto_direction_observer",
        "s_vehicle_direction_known = s_auto_direction_result.direction_known",
        "HallSpeed_SetCommandDirection(s_vehicle_direction)",
        "servo_basic_estimated_vehicle_direction() : 0",
    ]), "automatic Hall sign follows the separate AUTO feedback observer, not the requested target or gate phase")
    add(results, "hall_speed_telemetry_uses_coherent_snapshot", all(needle in hall_header_text for needle in [
        "HallSpeed_GetSnapshotSpeedMps(const hall_speed_state_t *snapshot",
    ]) and all(needle in data_text for needle in [
        "hall_snapshot = HallSpeed_GetState()",
        "HallSpeed_GetSnapshotSpeedMps(&hall_snapshot, &hall_speed_mps)",
        "STATUS_BIT_HALL_SPEED_VALID",
        "STATUS_BIT_HALL_STOP_CONFIRMED",
        "write_i32_be(&basebuffer[3], (int32_t)(hall_speed_mps * 1000.0f))",
    ]) and "hall_delta_count" not in data_text,
        "byte 3-6 Hall speed and its valid/stopped flags come from one Hall snapshot")
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
    add(results, "unknown_direction_invalid_speed", all(needle in text for needle in [
        "const int8_t direction = servo_basic_estimated_vehicle_direction()",
        "direction != 0)",
        "feedback_speed = 0.0f",
        "*yaw_rate_rad_s = (speed_valid != 0U) ? yaw_rate : 0.0f",
    ]), "unknown vehicle direction cannot produce signed speed or yaw")
    add(results, "esc_magnitude_status_separates_direction", all(needle in data_text for needle in [
        "ServoBasic_GetControlSnapshot(&control_snapshot)",
        "esc_speed_magnitude_valid = control_snapshot.esc_uplink_speed_valid",
        "esc_direction_known = control_snapshot.esc_direction_known",
        "speed_mps = control_snapshot.esc_uplink_speed_mps",
        "if (esc_speed_magnitude_valid != 0U)",
        "status_bits |= STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID",
        "status_bits |= STATUS_BIT_VEHICLE_DIRECTION_KNOWN",
        "servo_diagnostics.esc_stop_confirmed",
    ]) and all(needle in text for needle in [
        "diagnostics.esc_feedback_valid",
        "diagnostics.esc_speed_magnitude_valid",
        "diagnostics.vehicle_direction_known",
        "diagnostics.esc_feedback_direction",
        "s_vehicle_direction_known",
    ]), "bit6 reports ESC speed magnitude while bit23 separately reports known vehicle direction")
    add(results, "esc_uplink_low_gear_calibration", all(needle in vehicle_config_text for needle in [
        "#define APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT 0.14115f",
        "#define APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT      250U",
    ]) and all(needle in text for needle in [
        "g_esc_low_gear_wheel_rpm_per_raw",
        "servo_basic_compute_uplink_speed_magnitude",
        "config.wheel_rpm_per_raw = g_esc_low_gear_wheel_rpm_per_raw",
        "config.wheel_radius_m = (float)get_orin_ackermann_wheel_radius_mm() / 1000.0f",
        "*speed_magnitude_mps = s_esc_motion_estimate.speed_magnitude_mps",
        "diagnostics.esc_speed_calibration_valid =",
        "s_esc_motion_estimator.config_valid",
    ]), "uplink speed uses low-gear raw-rpm calibration and the existing vehicle wheel radius, independent of ESC motion gate config")
    add(results, "control_path_uses_compact_receiver_health", all(needle in text for needle in [
        "EscTelemetryReceiverHealth_t s_esc_receiver_health",
        "EscTelemetry_GetReceiverHealth(&s_esc_receiver_health)",
    ]) and "EscTelemetry_GetDiagnostics(" not in text and all(
        needle in esc_telemetry_text for needle in [
            "void EscTelemetry_GetReceiverHealth(",
            "health->samples_published = s_diagnostics.samples_published",
            "health->rx_error_count = s_diagnostics.rx_error_count",
            "health->last_rx_error_flags = s_diagnostics.last_rx_error_flags",
        ]
    ) and "esc_telemetry_sync_parser_diagnostics();" not in esc_telemetry_text.split(
        "void EscTelemetry_GetReceiverHealth(", 1
    )[1], "20 ms control path reads only compact receiver health without full diagnostic sync/copy")
    add(results, "control_snapshot_single_reader_source", all(needle in data_text for needle in [
        "servo_basic_control_snapshot_t control_snapshot",
        "ServoBasic_GetControlSnapshot(&control_snapshot)",
        "servo_diagnostics = control_snapshot.diagnostics",
        "speed_mps = control_snapshot.esc_uplink_speed_mps",
        "yaw_rate_rad_s = (control_snapshot.signed_speed_valid != 0U)",
    ]) and all(needle in show_text for needle in [
        "servo_basic_control_snapshot_t control_snapshot",
        "ServoBasic_GetControlSnapshot(&control_snapshot)",
        "show_runtime_page(&control_snapshot, &hall)",
        "show_diagnostic_page(&control_snapshot, &hall)",
        "control_snapshot->esc_uplink_speed_mps",
    ]) and all(needle not in data_text for needle in [
        "ServoBasic_GetState(",
        "ServoBasic_GetAckermannFeedback(",
    ]) and "ServoBasic_GetState(" not in show_text,
        "data and OLED tasks consume one published control snapshot instead of mixed-cycle getters")
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
    add(results, "esc_stop_telemetry_zeroes_wire_motion", matches(
        data_text,
        r"if\s*\(servo_diagnostics\.esc_stop_confirmed\s*!=\s*0U\)\s*\{"
        r".*?speed_mps\s*=\s*0\.0f;"
        r".*?yaw_rate_rad_s\s*=\s*0\.0f;"
        r".*?esc_speed_magnitude_valid\s*=\s*0U;"
        r".*?esc_direction_known\s*=\s*0U;",
    ), "ESC-confirmed stop clears wire speed, yaw, magnitude validity, and direction-known status")
    add(results, "esc_motion_and_standstill_status_exclusive", matches(
        data_text,
        r"if\s*\(servo_diagnostics\.esc_stop_confirmed\s*!=\s*0U\)\s*\{"
        r".*?STATUS_BIT_HALL_STANDSTILL_CONFIRMED;"
        r".*?\}\s*else\s+if\s*\(esc_speed_magnitude_valid\s*!=\s*0U\)\s*\{"
        r".*?STATUS_BIT_ESC_SPEED_MAGNITUDE_VALID;",
    ), "telemetry cannot mark ESC magnitude-valid motion and ESC-confirmed standstill together")
    add(results, "auto_per_frame_context_filters_source_purpose_session", all(needle in text for needle in [
        "EscTelemetry_ContextSource(item.output_context)",
        "EscTelemetry_ContextRcActive(item.output_context) != 0U",
        "EscTelemetry_MetadataAutoContext(item.output_metadata) == 0U",
        "EscTelemetry_MetadataPurpose(item->output_metadata)",
        "EscTelemetry_MetadataSession(item->output_metadata)",
        "matching_output = (session == s_mode2_drive_gate.current_session_id",
        "observation.context_purpose = purpose",
        "observation.context_session_id = session",
    ]) and all(needle in mode2_text for needle in [
        "context_session_id == gate->current_session_id",
        "context_purpose ==",
        "context_is_brake_session",
        "ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE",
    ]), "AUTO evidence is filtered per frame by source, purpose, and session")
    add(results, "auto_rejects_stale_or_old_feedback", all(needle in text for needle in [
        "s_auto_delivery_epoch != item.delivery_epoch",
        "servo_basic_tick_delta_ms(now_ms, item.sample.received_tick_ms, &age_ms)",
        "age_ms > s_esc_motion_config.telemetry_timeout_ms",
        "item.sample.sample_id == s_esc_last_observed_sample_id",
        "s_observation_diagnostics.samples_wrong_source++",
        "s_observation_diagnostics.samples_expired++",
        "servo_basic_invalidate_auto_history(now_ms)",
        "s_auto_fault_pending = 1U",
    ]) and all(needle in mode2_text for needle in [
        "observation_is_usable_for_evidence",
        "tick_at_or_after(observation->sample_tick_ms",
        "is_new_sample(gate, observation) == 0U",
    ]), "AUTO rejects old-session, wrong-source, duplicate, stale, or time-regressing feedback before it can prove state")
    add(results, "brake_pwm_mapping_change_invalidates_history", all(needle in text for needle in [
        "config.center_pwm_us = (g_orin_esc_center_us <= UINT16_MAX)",
        "config.brake_min_us = (g_auto_brake_min_us <= UINT16_MAX)",
        "Electrical calibration cannot change an active continuous session.",
        "s_auto_authorized_previous != 0U || s_esc_stop_confirmed == 0U",
        "s_mode2_drive_config.center_pwm_us != gate.center_pwm_us",
        "servo_basic_invalidate_auto_history(now_ms)",
    ]), "center/brake calibration changes invalidate history and cannot mutate an active continuous session")
    add(results, "speed_pid_uses_committed_sample_span_not_frame_delta", all(needle in longitudinal_text for needle in [
        "pid_sample_committed",
        "pid_committed_sample_tick_ms",
        "latest_sample_is_committed(controller)",
        "controller->pending_dt_s = dt_s",
        "controller->pending_new_sample",
        "LongitudinalController_CommitApplied",
        "controller->pid_committed_sample_id = controller->feedback_sample_id",
    ]),
        "PID integration spans the last committed feedback sample to the latest evaluated sample")
    add(results, "speed_pid_clears_integral_on_target_sign_change_without_ramp", all(needle in longitudinal_text for needle in [
        "target_sign_changed",
        "controller->skip_error_integral_once = 1U",
        "target_direction != controller->last_target_direction",
        "controller->integral_us = 0.0f",
    ]) and all(needle not in longitudinal_text for needle in [
        "slewed_target_mps",
        "target_slew",
    ]), "PID has no signed speed ramp; target changes skip one I update and sign changes clear integral")
    add(results, "actual_output_antiwindup", all(needle in text for needle in [
        "applied.applied_output_us = (float)g_state.esc_pulse_us - (float)get_orin_esc_center_pulse()",
        "applied.min_output_us = s_mode2_drive_output.pid_min_us",
        "applied.max_output_us = s_mode2_drive_output.pid_max_us",
        "applied.pid_active = s_mode2_drive_output.pid_active",
        "applied.reset_integral = s_mode2_drive_output.integral_reset",
        "LongitudinalController_CommitApplied(&s_longitudinal_controller, &applied)",
    ]) and all(needle in longitudinal_text for needle in [
        "applied_output_us - controller->pending_raw_output_us",
        "beta *",
        "controller->config.antiwindup_tau_s",
        "controller->config.ki_us_per_mps_s == 0.0f",
    ]), "PID anti-windup uses the actual quantized output and gate-provided bounds")
    add(results, "mode2_qualification_uses_physical_output_only_fwd_to_rev", all(needle in text for needle in [
        "Mode2DriveGate_CommitApplied(&s_mode2_drive_gate, &s_mode2_drive_output",
        "EscTelemetry_PrepareOutputContext(pulse_us",
        "EscTelemetry_CommitOutputContext(&prepared)",
    ]) and all(needle in mode2_text for needle in [
        "MODE2_DRIVE_ACTION_FWD_TO_REV_BRAKE",
        "MODE2_DRIVE_BRAKE_PURPOSE_REVERSE",
        "pwm_us <= gate->config.reverse_pwm_us",
        "gate->full_pwm_active = 1U",
        "observation->context_pwm_us <= gate->config.reverse_pwm_us",
        "gate->brake_full_confirmed = 1U",
        "stopped_after(gate, gate->brake_full_start_ms)",
    ]) and all(needle not in mode2_text for needle in [
        "REV_TO_FWD_BRAKE",
    ]), "only forward-to-reverse uses full physical brake qualification from applied PWM evidence")
    add(results, "reverse_deceleration_coasts_without_positive_brake", all(needle in mode2_text for needle in [
        "static Mode2DriveGateOutput_t evaluate_reverse_ready",
        "MODE2_DRIVE_PHASE_R_COAST",
        "MODE2_DRIVE_REASON_REVERSE_COAST",
        "return neutral_output(gate,",
    ]) and "ESC_TELEMETRY_OUTPUT_PURPOSE_FORWARD_BRAKE" not in mode2_text.split(
        "static Mode2DriveGateOutput_t evaluate_reverse_ready", 1
    )[1].split("static uint8_t neutral_dwell_ready", 1)[0],
        "reverse deceleration and reverse-to-forward use neutral coast, never positive braking")
    add(results, "no_motor_startup_timeout_or_fake_direction", all(needle in mode2_text for needle in [
        "MODE2_DRIVE_REASON_STARTUP_HOLD",
        "startup_hold_output",
        "gate->latest_observation.context_session_id ==",
        "gate->current_session_id",
        "elapsed_ms(now_ms, gate->last_commit_ms) <=",
        "MODE2_DRIVE_REASON_DIRECTION_UNKNOWN",
    ]) and all(needle not in mode2_text for needle in [
        "FORWARD_RECOVERY_TIMEOUT",
        "MOTOR_STARTUP_TIMEOUT",
        "mechanical_response",
    ]),
        "startup holds only an existing same-session request; there is no motor-response timeout or fabricated direction")
    return results


def check_vehicle_defaults(root: Path) -> list[Check]:
    text = read_text(root, "WHEELTEC_APP/Inc/app_vehicle_config.h")
    control_text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    longitudinal_text = read_text(root, "WHEELTEC_APP/longitudinal_controller.c")
    mode2_text = read_text(root, "WHEELTEC_APP/mode2_drive_gate.c")
    cmake_text = read_text(root, "CMakeLists.txt")
    keil_text = read_text(root, "MDK-ARM/WHEELTEC.uvprojx")
    results: list[Check] = []
    expected = [
        "#define APP_ORIN_PWM_TIMEOUT_DEFAULT_MS           250U",
        "#define APP_ORIN_ACKERMANN_WHEELBASE_MM           600U",
        "#define APP_ORIN_ACKERMANN_TRACK_WIDTH_MM         500U",
        "#define APP_ORIN_ACKERMANN_WHEEL_RADIUS_MM        115U",
        "#define APP_ORIN_ACKERMANN_MAX_STEERING_MRAD      349U",
        "#define APP_ESC_LOW_GEAR_WHEEL_RPM_PER_RAW_DEFAULT 0.14115f",
        "#define APP_ESC_SPEED_FRESH_TIMEOUT_MS_DEFAULT      250U",
        "#define APP_HALL_GLITCH_FAULT_CONFIRM_EVENTS         3U",
        "#define APP_ESC_STOPPED_THRESHOLD_MPS_DEFAULT      0.05f",
        "#define APP_ESC_STOPPED_MIN_SAMPLES_DEFAULT           3U",
        "#define APP_ESC_STOPPED_MIN_COVERAGE_MS_DEFAULT     150U",
        "#define APP_ORIN_STEERING_RATE_LIMIT_MRADPS       900U",
        "#define APP_SPEED_PID_KP_DEFAULT_US_PER_MPS       100.0f",
        "#define APP_SPEED_PID_KI_DEFAULT_US_PER_MPS_S       0.0f",
        "#define APP_SPEED_PID_KD_DEFAULT_US_PER_MPS2        0.0f",
        "#define APP_SPEED_PID_DERIVATIVE_TAU_MS              80U",
        "#define APP_SPEED_PID_TRACKING_TAU_MS               100U",
        "#define APP_AUTO_BRAKE_MIN_US                        50U",
        "#define APP_AUTO_BRAKE_ENTER_ERROR_MPS              0.20f",
        "#define APP_AUTO_BRAKE_ENTER_RATIO                  0.10f",
        "#define APP_AUTO_BRAKE_RELEASE_ERROR_MPS            0.05f",
        "#define APP_AUTO_BRAKE_RELEASE_RATIO                0.02f",
        "#define APP_AUTO_COAST_EVAL_MS                       200U",
        "#define APP_AUTO_COAST_BUDGET_MS                     600U",
        "#define APP_AUTO_A_PROGRESS_MPS2                   0.05f",
        "#define APP_AUTO_BRAKE_RELEASE_DELAY_MS               80U",
        "#define APP_AUTO_ACTION_ACK_MS                      300U",
        "#define APP_AUTO_QUALIFY_MS                         100U",
        "#define APP_AUTO_NEUTRAL_DWELL_MS                   100U",
        "#define APP_ORIN_ESC_CENTER_US                   1500U",
        "#define APP_ORIN_SERVO_CENTER_US                 1500U",
        "#define APP_ORIN_SERVO_RANGE_US                   395U",
        "#define APP_ORIN_STEERING_PWM_DIRECTION_SIGN        (+1)",
    ]
    obsolete_defaults = [
        "APP_ESC_TRACKING_BRAKE_KP_DEFAULT",
        "APP_ESC_TRACKING_BRAKE_MAX_DEFAULT",
        "APP_MODE2_FWD_TO_REV_BRAKE_REQUEST_DEFAULT",
        "APP_MODE2_REV_TO_FWD_BRAKE_REQUEST_DEFAULT",
        "APP_ORIN_ACCEL_LIMIT_MMPS2",
    ]
    add(results, "vehicle_defaults", all(needle in text for needle in expected) and all(needle not in text for needle in obsolete_defaults), "confirmed geometry, servo calibration, PID gains, AUTO brake timing, and ESC/mode2 defaults")
    add(
        results,
        "longitudinal_full_pid_without_feedforward_or_ramp",
        all(needle in longitudinal_text for needle in [
            "kp_us_per_mps",
            "ki_us_per_mps_s",
            "kd_us_per_mps2",
            "LongitudinalController_ObserveFeedback",
            "LongitudinalController_Evaluate",
            "LongitudinalController_CommitApplied",
            "finite_positive(config->kp_us_per_mps)",
            "finite_positive(config->derivative_tau_s)",
            "finite_nonnegative(config->ki_us_per_mps_s)",
            "finite_nonnegative(config->kd_us_per_mps2)",
            "pending_raw_output_us",
            "pending_dt_s",
            "applied_output_bounds_are_valid",
        ]) and all(needle not in control_text for needle in [
            "s_orin_forward_ff_table",
            "s_orin_reverse_ff_table",
            "interpolate_speed_ff_table",
        ]) and all(needle not in longitudinal_text for needle in [
            "s_forward_ff_table",
            "s_reverse_ff_table",
            "feedforward_pwm_us",
            "slewed_target_mps",
            "target_slew",
            "LONGITUDINAL_INTENT_",
            "drive_pwm_us",
            "tracking_brake",
        ]),
        "longitudinal controller is full PID and contains no feedforward table, target ramp, or legacy drive intent",
    )
    add(
        results,
        "no_artificial_speed_caps_or_deadband",
        all(needle not in text for needle in [
            "MIN_COMMAND_SPEED",
            "VX_FORWARD_CAP",
            "VX_REVERSE_CAP",
            "VX_DEADBAND",
            "SPEED_LIMIT_MMPS",
        ]) and all(needle not in control_text for needle in [
            "scale_and_limit_orin_vx",
            "orin_target_is_below_min_control",
            "esc_speed_limit_protection_active",
        ]) and "clamp_abs_target" not in longitudinal_text,
        "speed commands have no software deadband or speed cap; final PWM bounds remain",
    )
    add(
        results,
        "auto_brake_scheduled_by_gate_not_pid_core",
        all(needle in mode2_text for needle in [
            "forward_needs_brake",
            "e_on_abs_mps",
            "e_on_ratio",
            "e_off_abs_mps",
            "e_off_ratio",
            "coast_eval_ms",
            "coast_budget_ms",
            "forward_brake_output",
            "MODE2_DRIVE_BRAKE_PURPOSE_TRACK",
            "track_brake_release_ready",
        ]) and all(needle in control_text for needle in [
            "g_auto_brake_enter_error_mps",
            "g_auto_brake_enter_ratio",
            "g_auto_brake_release_error_mps",
            "g_auto_brake_release_ratio",
            "g_auto_coast_eval_ms",
            "g_auto_coast_budget_ms",
            "g_auto_a_progress_mps2",
        ]) and all(needle not in longitudinal_text for needle in [
            "tracking_brake",
            "LONGITUDINAL_INTENT_TRACKING_BRAKE",
        ]),
        "Mode2 gate schedules AUTO braking from PID error/progress; PID core remains continuous full PID",
    )
    add(
        results,
        "pid_offsets_are_mapped_to_physical_pwm_by_gate",
        all(needle in mode2_text for needle in [
            "static int32_t forward_span",
            "static int32_t reverse_span",
            "static uint16_t pwm_from_offset",
            "static Mode2DriveGateOutput_t forward_output",
            "static Mode2DriveGateOutput_t reverse_output",
            "clip_f32(raw_pid_us, 0.0f, high)",
            "clip_f32(raw_pid_us, low, 0.0f)",
            "output.pid_min_us = 0.0f",
            "output.pid_max_us = high",
            "output.pid_min_us = low",
            "output.pid_max_us = 0.0f",
        ]) and all(needle in control_text for needle in [
            "config.min_output_us = (float)ESC_PWM_MIN_PULSE_US - (float)s_active_esc_center",
            "config.max_output_us = (float)ESC_PWM_MAX_PULSE_US - (float)s_active_esc_center",
            "applied.applied_output_us = (float)g_state.esc_pulse_us - (float)get_orin_esc_center_pulse()",
            "applied.min_output_us = s_mode2_drive_output.pid_min_us",
            "applied.max_output_us = s_mode2_drive_output.pid_max_us",
        ]),
        "PID returns signed offset; Mode2 gate maps offsets to physical PWM and returns current phase bounds for anti-windup",
    )
    add(
        results,
        "brake_pwm_endpoint_direction_guard",
        all(needle in mode2_text for needle in [
            "config->reverse_pwm_us >= config->center_pwm_us",
            "config->forward_pwm_us <= config->center_pwm_us",
            "config->brake_min_us > config->center_pwm_us - config->reverse_pwm_us",
            "valid_duration(config->qualify_ms)",
            "valid_duration(config->neutral_dwell_ms)",
            "config->coast_eval_ms >= config->coast_budget_ms",
            "finite_positive(config->e_on_abs_mps)",
            "config->e_off_abs_mps >= config->e_on_abs_mps",
        ]),
        "Mode2 validates asymmetric endpoint ordering, brake strength, timing, and hysteresis parameters",
    )
    add(
        results,
        "longitudinal_controller_in_both_projects",
        all(needle in cmake_text for needle in [
            "WHEELTEC_APP/longitudinal_controller.c",
            "test_longitudinal_controller",
        ]) and all(needle in keil_text for needle in [
            "<FileName>longitudinal_controller.c</FileName>",
            "<FilePath>..\\WHEELTEC_APP\\longitudinal_controller.c</FilePath>",
        ]),
        "CMake host/ARM and Keil projects compile the longitudinal controller",
    )
    add(
        results,
        "rc_direction_observer_in_both_projects",
        all(needle in cmake_text for needle in [
            "WHEELTEC_APP/rc_direction_observer.c",
            "test_rc_direction_observer",
            "target_link_libraries(WHEELTEC.elf PRIVATE",
            "rc_direction_observer",
        ]) and all(needle in keil_text for needle in [
            "<FileName>rc_direction_observer.c</FileName>",
            "<FilePath>..\\WHEELTEC_APP\\rc_direction_observer.c</FilePath>",
        ]),
        "CMake host/ARM and Keil projects compile the RC direction observer",
    )
    add(
        results,
        "missing_esc_config_disables_auto_propulsion",
        all(needle in control_text for needle in [
            "servo_basic_auto_propulsion_authorized",
            "s_esc_motion_estimate.config_valid",
            "s_esc_motion_estimate.stop_valid",
            "s_esc_feedback_available",
            "servo_basic_mode2_application_config_valid()",
            "s_longitudinal_controller.config_valid",
            "pid_input.enabled = servo_basic_auto_propulsion_authorized()",
            "gate_input.propulsion_authorized = servo_basic_auto_propulsion_authorized()",
        ]),
        "invalid drivetrain or mode2 config keeps automatic propulsion unauthorized",
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
    add(results, "neutral_stop_and_timeout", all(needle in text for needle in [
        "apply_esc_pulse(get_orin_esc_center_pulse())",
        "apply_servo_pulse(get_orin_servo_center_pulse())",
        "pulse_us = get_orin_esc_center_pulse();",
        "limit_servo_safe_pulse(clamp_servo_pulse(g_orin_state.servo_pulse_us)) :",
        "get_orin_servo_center_pulse());",
        "g_orin_state.software_stop == 0U",
        "s_mode2_drive_output.purpose == ESC_TELEMETRY_OUTPUT_PURPOSE_NEUTRAL",
        "s_mode2_drive_gate.config_valid == 0U",
    ]), "software stop, timeout, neutral, and invalid-config paths use configured centers")
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
        "set_rc_override_state(1U, 1U)",
        "set_rc_override_state(1U, 0U)",
    ]), "idle RC passthrough releases immediately, but a real manual override keeps the 500 ms hold")
    add(results, "manual_rc_throttle_preserves_full_receiver_travel", all(needle in text for needle in [
        "A valid receiver throttle pulse is already bounded by the capture",
        "return pulse_us;",
        "const uint16_t esc_pulse = (g_rc_throttle_present != 0U) ?",
        "rc_select_pulse(g_rc_throttle_current, 1U)",
    ]), "manual RC throttle bypasses automatic propulsion endpoints and preserves the validated receiver pulse")
    return results


def check_fault_recovery(root: Path) -> list[Check]:
    servo_text = read_text(root, "WHEELTEC_APP/servo_basic_control.c")
    rc_capture_text = read_text(root, "WHEELTEC_APP/servo_rc_capture.c")
    rc_capture_header_text = read_text(root, "WHEELTEC_APP/Inc/servo_rc_capture.h")
    show_text = read_text(root, "WHEELTEC_APP/show_task.c")
    main_text = read_text(root, "Core/Src/main.c")
    tim_text = read_text(root, "Core/Src/tim.c")
    ioc_text = read_text(root, "WHEELTEC.ioc")
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
    add(results, "pd14_is_unconfigured", all(needle in main_text for needle in [
        "HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1)",
        "HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2)",
    ]) and all(needle not in "\n".join([
        servo_text,
        rc_capture_text,
        rc_capture_header_text,
        show_text,
        main_text,
        tim_text,
        ioc_text,
    ]) for needle in [
        "ServoRC_GetAuxPulse",
        "ServoRC_IsAuxActive",
        "ServoRC_HasAuxFault",
        "g_rc_aux",
        "HAL_TIM_ACTIVE_CHANNEL_3",
        "TIM_CHANNEL_3",
        "S_TIM4_CH3",
        "PD14.Signal",
    ]), "unconnected PD14 has no guard/AUX API, TIM4 CH3 setup, or runtime state")
    add(results, "transient_rc_glitch_not_reported_live", contains(
        servo_text, "diagnostics.steering_fault = (g_rc_input_fault_active != 0U) ? 1U : 0U;"
    ), "diagnostics report the persistent RC input fault, not the immediate glitch watch value")
    return results


def check_uart(root: Path) -> list[Check]:
    text = read_text(root, "Core/Src/usart.c")
    main_header_text = read_text(root, "Core/Inc/main.h")
    main_text = read_text(root, "Core/Src/main.c")
    dma_text = read_text(root, "Core/Src/dma.c")
    gpio_text = read_text(root, "Core/Src/gpio.c")
    irq_text = read_text(root, "Core/Src/stm32f4xx_it.c")
    soft_uart_text = read_text(root, "WHEELTEC_APP/esc_soft_uart_stm32.c")
    telemetry_stm32_text = read_text(root, "WHEELTEC_APP/esc_telemetry_stm32.c")
    show_text = read_text(root, "WHEELTEC_APP/show_task.c")
    ioc_text = read_text(root, "WHEELTEC.ioc")
    results: list[Check] = []
    add(results, "uart4_instance", contains(text, "huart4.Instance = UART4"), "UART4 instance")
    add(results, "uart4_115200_8n1", all(needle in text for needle in [
        "huart4.Init.BaudRate = 115200",
        "huart4.Init.WordLength = UART_WORDLENGTH_8B",
        "huart4.Init.StopBits = UART_STOPBITS_1",
        "huart4.Init.Parity = UART_PARITY_NONE",
    ]), "UART4 115200 8N1")
    add(results, "uart4_tx_rx", contains(text, "huart4.Init.Mode = UART_MODE_TX_RX"), "UART4 TX/RX mode")
    add(results, "esc_rx_no_usart2_runtime", all(needle not in main_text for needle in [
        "MX_USART2_UART_Init",
    ]) and all(needle not in text for needle in [
        "huart2.Instance = USART2",
        "GPIO_AF7_USART2",
        "hdma_usart2_rx",
        "DMA1_Stream5",
    ]) and all(needle not in dma_text for needle in [
        "DMA1_Stream5_IRQn",
    ]) and all(needle not in irq_text for needle in [
        "USART2_IRQHandler",
        "hdma_usart2_rx",
    ]), "ESC RX runtime no longer initializes PA3/USART2/DMA1_Stream5")
    add(results, "esc_rx_pd15_gpio_exti", all(needle in gpio_text for needle in [
        "EscSoftUartRx_Pin",
        "GPIO_MODE_IT_FALLING",
        "GPIO_PULLUP",
        "HAL_NVIC_SetPriority(EXTI15_10_IRQn, 4, 0)",
    ]) and all(needle in ioc_text for needle in [
        "PD15.GPIO_Label=EscSoftUartRx",
        "PD15.GPIO_Mode=GPIO_MODE_IT_FALLING",
        "PD15.GPIO_PuPd=GPIO_PULLUP",
        "PD15.Signal=GPIO_EXTI15",
    ]), "PD15 is the ESC RX EXTI input with pull-up")
    add(results, "esc_rx_tim5_sampler", all(needle in soft_uart_text for needle in [
        "__HAL_RCC_TIM5_CLK_ENABLE()",
        "HAL_NVIC_SetPriority(TIM5_IRQn, 3U, 0U)",
        "TIM5->PSC = 0UL",
        "TIM5->ARR = 0xFFFFFFFFUL",
        "s_sample_offsets",
        "uwTick",
    ]) and all(needle in irq_text for needle in [
        "EscSoftUartStm32_HandleExti15Irq",
        "EscSoftUartStm32_HandleTim5Irq",
        "void TIM5_IRQHandler(void)",
    ]), "PD15 software UART uses TIM5 priority-3 direct sampling")
    add(results, "esc_rx_start_glitch_nonfatal", all(needle in soft_uart_text for needle in [
        "static void esc_soft_uart_filter_start_glitch(void)",
        "s_diagnostics.start_glitches++",
        "esc_soft_uart_enable_exti()",
        "result->fault_flags == ESC_SOFT_UART_RX_FAULT_START_GLITCH",
    ]) and not matches(
        soft_uart_text,
        r"esc_soft_uart_set_fault\s*\(\s*ESC_SOFT_UART_RX_FAULT_START_GLITCH\s*\)",
    ), "false start glitches are counted and rearmed without setting the fatal fault mask")
    add(results, "esc_rx_task_feeds_parser", all(needle in telemetry_stm32_text for needle in [
        "EscSoftUartStm32_TakeFaults",
        "EscTelemetry_RecordRxError",
        "EscSoftUartStm32_ReadByte",
        "EscTelemetry_ProcessReceivedByte",
        "item.received_tick_ms",
        "item.output_context",
        "xTaskNotifyGive(g_servoTaskHandle)",
    ]), "soft UART preserves byte time/context, faults invalidate epoch, and completed frames notify Servo")
    add(results, "oled_pd3_level_selected_pages", all(needle in main_header_text for needle in [
        "#define UserKey_Pin GPIO_PIN_3",
        "#define UserKey_GPIO_Port GPIOD",
    ]) and all(needle in gpio_text for needle in [
        "GPIO_InitStruct.Pin = UserKey_Pin",
        "GPIO_InitStruct.Mode = GPIO_MODE_INPUT",
        "HAL_GPIO_Init(UserKey_GPIO_Port, &GPIO_InitStruct)",
    ]) and all(needle in ioc_text for needle in [
        "PD3.GPIO_Label=UserKey",
        "PD3.GPIO_PuPd=GPIO_PULLUP",
        "PD3.Signal=GPIO_Input",
    ]) and all(needle in show_text for needle in [
        "#define SHOW_KEY_POLL_MS       20U",
        "#define SHOW_KEY_DEBOUNCE_MS   40U",
        "show_runtime_page",
        "show_diagnostic_page",
        "HAL_GPIO_ReadPin(UserKey_GPIO_Port, UserKey_Pin)",
        "(stable_level == 0U) ?",
        "SHOW_PAGE_RUNTIME : SHOW_PAGE_DIAGNOSTIC",
    ]) and show_text.count("control_snapshot->state.esc_pulse_us") >= 2
        and show_text.count("control_snapshot->state.servo_pulse_us") >= 2,
        "PD3 selects debounced OLED pages and both pages retain live ESC/servo PWM")
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
        + check_speed_feedback_sources(root)
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
