#!/usr/bin/env python3
"""PC-side UART4 Ackermann smoke check for the current firmware contract.

Default mode is passive and never sends a motion command. Modes that can move
the car require --arm-motion when speed or steering are non-zero.
"""

from __future__ import annotations

import argparse
import math
import os
import select
import struct
import sys
import termios
import time
import tty
from dataclasses import dataclass


FRAME_HEADER = 0x7B
FRAME_TAIL = 0x7D
COMMAND_ACKERMANN = 0x01
COMMAND_SIZE = 11
TELEMETRY_SIZE = 24
TELEMETRY_PROTOCOL_ID = 0xA1
MIN_COMMAND_SPEED_MPS = 0.3
MAX_COMMAND_SPEED_MPS = 3.0
MAX_STEERING_ANGLE_RAD = 0.262

FLAG_ENABLE = 1 << 0
FLAG_SOFTWARE_STOP = 1 << 7
STATUS_HALL_FEEDBACK_VALID = 1 << 6
STATUS_HALL_STANDSTILL_CONFIRMED = 1 << 12


@dataclass
class Telemetry:
    status_flags: int
    seq: int
    hall_delta_count: int
    speed_mps: float
    steering_angle_rad: float
    yaw_rate_radps: float
    battery_v: float
    dt_ms: int
    status_bits: int
    protocol_id: int


class TelemetryContractError(ValueError):
    """A structurally valid telemetry frame violates the firmware contract."""


def checksum(data: bytes | bytearray) -> int:
    value = 0
    for byte in data:
        value ^= byte
    return value & 0xFF


def quantize_milli(value: float) -> int:
    """Match C/C++ round-to-nearest with half values away from zero."""
    scaled = value * 1000.0
    if scaled >= 0.0:
        return math.floor(scaled + 0.5)
    return math.ceil(scaled - 0.5)


def build_command_frame(
    speed_mps: float,
    steering_angle_rad: float,
    *,
    enable: bool,
    software_stop: bool,
) -> bytes:
    if not math.isfinite(speed_mps) or not math.isfinite(steering_angle_rad):
        raise ValueError("speed and steering must be finite")
    if abs(speed_mps) > MAX_COMMAND_SPEED_MPS:
        raise ValueError("speed exceeds the confirmed +/-3.0 m/s command boundary")
    if abs(steering_angle_rad) > MAX_STEERING_ANGLE_RAD:
        raise ValueError("steering exceeds the confirmed +/-0.262 rad boundary")

    flags = 0
    if enable:
        flags |= FLAG_ENABLE
    if software_stop:
        flags |= FLAG_SOFTWARE_STOP
        speed_mps = 0.0
        steering_angle_rad = 0.0
    if not enable:
        speed_mps = 0.0
    if 0.0 < abs(speed_mps) < MIN_COMMAND_SPEED_MPS:
        speed_mps = 0.0

    speed_mmps = quantize_milli(speed_mps)
    steering_mrad = quantize_milli(steering_angle_rad)
    frame = bytearray(COMMAND_SIZE)
    frame[0] = FRAME_HEADER
    frame[1] = COMMAND_ACKERMANN
    frame[2] = flags
    frame[3:5] = struct.pack(">h", int(speed_mmps))
    frame[5:7] = struct.pack(">h", int(steering_mrad))
    frame[7:9] = b"\x00\x00"
    frame[9] = checksum(frame[:9])
    frame[10] = FRAME_TAIL
    return bytes(frame)


def parse_telemetry(frame: bytes) -> Telemetry:
    if len(frame) != TELEMETRY_SIZE:
        raise ValueError(f"expected {TELEMETRY_SIZE} bytes, got {len(frame)}")
    if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        raise ValueError("bad telemetry header or tail")
    if checksum(frame[:22]) != frame[22]:
        raise ValueError("bad telemetry BCC")
    if frame[21] != TELEMETRY_PROTOCOL_ID:
        raise ValueError(
            f"unexpected telemetry protocol id 0x{frame[21]:02X}; expected 0x{TELEMETRY_PROTOCOL_ID:02X}"
        )

    speed_mps = struct.unpack(">h", frame[7:9])[0] / 1000.0
    status_bits = struct.unpack(">I", frame[17:21])[0]
    hall_valid = bool(status_bits & STATUS_HALL_FEEDBACK_VALID)
    hall_standstill = bool(status_bits & STATUS_HALL_STANDSTILL_CONFIRMED)
    if hall_valid and hall_standstill:
        raise TelemetryContractError(
            "Hall motion-valid and standstill-confirmed bits are both set"
        )
    if hall_valid and speed_mps == 0.0:
        raise TelemetryContractError("Hall motion-valid status carries a zero speed")
    if not hall_valid and speed_mps != 0.0:
        raise TelemetryContractError(
            "Hall speed is non-zero without motion-valid status"
        )

    return Telemetry(
        status_flags=frame[1],
        seq=frame[2],
        hall_delta_count=struct.unpack(">i", frame[3:7])[0],
        speed_mps=speed_mps,
        steering_angle_rad=struct.unpack(">h", frame[9:11])[0] / 1000.0,
        yaw_rate_radps=struct.unpack(">h", frame[11:13])[0] / 1000.0,
        battery_v=struct.unpack(">H", frame[13:15])[0] / 1000.0,
        dt_ms=struct.unpack(">H", frame[15:17])[0],
        status_bits=status_bits,
        protocol_id=frame[21],
    )


def configure_serial(fd: int, baud: int) -> list[int | bytes]:
    baud_map = {
        9600: termios.B9600,
        19200: termios.B19200,
        38400: termios.B38400,
        57600: termios.B57600,
        115200: termios.B115200,
    }
    if baud not in baud_map:
        raise ValueError(f"unsupported baud rate {baud}")

    old_attrs = termios.tcgetattr(fd)
    tty.setraw(fd)
    attrs = termios.tcgetattr(fd)
    attrs[4] = baud_map[baud]
    attrs[5] = baud_map[baud]
    attrs[2] |= termios.CLOCAL | termios.CREAD
    attrs[2] &= ~termios.CSTOPB
    attrs[2] &= ~termios.PARENB
    attrs[2] &= ~termios.CSIZE
    attrs[2] |= termios.CS8
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return old_attrs


def read_frames(fd: int, duration_s: float) -> tuple[list[Telemetry], int]:
    deadline = time.monotonic() + duration_s
    buffer = bytearray()
    frames: list[Telemetry] = []
    contract_errors = 0

    while time.monotonic() < deadline:
        timeout = max(0.0, min(0.2, deadline - time.monotonic()))
        readable, _, _ = select.select([fd], [], [], timeout)
        if not readable:
            continue
        try:
            chunk = os.read(fd, 256)
        except BlockingIOError:
            continue
        if not chunk:
            continue
        buffer.extend(chunk)

        while True:
            try:
                start = buffer.index(FRAME_HEADER)
            except ValueError:
                buffer.clear()
                break
            if start:
                del buffer[:start]
            if len(buffer) < TELEMETRY_SIZE:
                break
            candidate = bytes(buffer[:TELEMETRY_SIZE])
            try:
                frames.append(parse_telemetry(candidate))
                del buffer[:TELEMETRY_SIZE]
            except TelemetryContractError as exc:
                contract_errors += 1
                print(f"ERROR rejected telemetry contract: {exc}", file=sys.stderr)
                # Header, tail, BCC, and protocol id were already valid.
                del buffer[:TELEMETRY_SIZE]
            except ValueError as exc:
                print(f"WARN rejected telemetry: {exc}", file=sys.stderr)
                del buffer[0]

    return frames, contract_errors


def print_frames(frames: list[Telemetry], contract_errors: int) -> None:
    if not frames:
        print("FAIL telemetry: no valid 24-byte telemetry frames received")
        return
    for frame in frames[-5:]:
        print(
            "TELEMETRY "
            f"seq={frame.seq} speed_mps={frame.speed_mps:.3f} "
            f"steering_rad={frame.steering_angle_rad:.3f} "
            f"yaw_rate_radps={frame.yaw_rate_radps:.3f} "
            f"battery_v={frame.battery_v:.2f} dt_ms={frame.dt_ms} "
            f"hall_valid={bool(frame.status_bits & STATUS_HALL_FEEDBACK_VALID)} "
            f"hall_standstill={bool(frame.status_bits & STATUS_HALL_STANDSTILL_CONFIRMED)} "
            f"status_flags=0x{frame.status_flags:02X} status_bits=0x{frame.status_bits:08X} "
            f"protocol_id=0x{frame.protocol_id:02X}"
        )
    if contract_errors:
        print(
            f"FAIL telemetry contract: rejected {contract_errors} inconsistent frame(s)"
        )
    else:
        print(f"PASS telemetry: received {len(frames)} valid frames")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port", required=True, help="stable serial device, normally /dev/autoracer_rc_chassis"
    )
    parser.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    parser.add_argument("--duration", type=float, default=3.0, help="seconds to listen after sending")
    parser.add_argument(
        "--mode",
        choices=("listen", "zero", "stop", "command"),
        default="listen",
        help="listen sends nothing; zero/stop send non-motion software commands",
    )
    parser.add_argument("--speed-mps", type=float, default=0.0, help="command mode speed")
    parser.add_argument("--steering-angle-rad", type=float, default=0.0, help="command mode steering")
    parser.add_argument("--arm-motion", action="store_true", help="required for non-zero command mode")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sends_motion = args.mode == "command" and (
        abs(args.speed_mps) > 1e-6 or abs(args.steering_angle_rad) > 1e-6
    )
    if sends_motion and not args.arm_motion:
        print("FAIL refusing non-zero command without --arm-motion", file=sys.stderr)
        return 2

    fd = os.open(args.port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old_attrs = configure_serial(fd, args.baud)
    try:
        if args.mode != "listen":
            if args.mode == "zero":
                frame = build_command_frame(0.0, 0.0, enable=True, software_stop=False)
            elif args.mode == "stop":
                frame = build_command_frame(0.0, 0.0, enable=False, software_stop=True)
            else:
                frame = build_command_frame(
                    args.speed_mps,
                    args.steering_angle_rad,
                    enable=True,
                    software_stop=False,
                )
            os.write(fd, frame)
            print(f"SENT {args.mode}: {frame.hex(' ')}")

        frames, contract_errors = read_frames(fd, args.duration)
        print_frames(frames, contract_errors)
        return 0 if frames and contract_errors == 0 else 1
    finally:
        termios.tcsetattr(fd, termios.TCSANOW, old_attrs)
        os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
