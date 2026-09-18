#!/usr/bin/env python3
"""Validate the GNU Arm ELF vector table for the C63A STM32F407VET6."""

import argparse
from pathlib import Path
import struct
import subprocess
import sys
import tempfile


def check(condition, message):
    if not condition:
        raise ValueError(message)


def check_vectors(elf):
    with tempfile.TemporaryDirectory(prefix="rccar-vectors-") as scratch:
        raw = Path(scratch) / "vectors.bin"
        subprocess.run(
            ["arm-none-eabi-objcopy", "-O", "binary",
             "--only-section=.isr_vector", str(elf), str(raw)],
            check=True,
        )
        data = raw.read_bytes()

    check(len(data) == 98 * 4, f"expected 98 vectors, got {len(data)} bytes")
    vectors = struct.unpack("<98I", data)
    check(vectors[0] == 0x20020000, f"unexpected initial SP: {vectors[0]:#x}")
    reserved = {7, 8, 9, 10, 13, 95}
    for index in reserved:
        check(vectors[index] == 0, f"reserved vector {index} must be zero")
    for index, value in enumerate(vectors):
        if index == 0 or index in reserved:
            continue
        check(value & 1, f"vector {index} is missing the Thumb bit: {value:#x}")
        check(0x08000000 <= (value & ~1) < 0x08080000,
              f"vector {index} is outside 512 KiB Flash: {value:#x}")

    symbols = {}
    output = subprocess.check_output(
        ["arm-none-eabi-nm", "--defined-only", str(elf)], text=True,
    )
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3:
            symbols[parts[2]] = int(parts[0], 16)
    handlers = {
        1: "Reset_Handler",
        2: "NMI_Handler",
        3: "HardFault_Handler",
        4: "MemManage_Handler",
        5: "BusFault_Handler",
        6: "UsageFault_Handler",
        11: "SVC_Handler",
        12: "DebugMon_Handler",
        14: "PendSV_Handler",
        15: "SysTick_Handler",
        31: "DMA1_Stream4_IRQHandler",
        46: "TIM4_IRQHandler",
        53: "USART1_IRQHandler",
        56: "EXTI15_10_IRQHandler",
        66: "TIM5_IRQHandler",
        68: "UART4_IRQHandler",
        71: "TIM7_IRQHandler",
        72: "DMA2_Stream0_IRQHandler",
        86: "DMA2_Stream7_IRQHandler",
    }
    for index, name in handlers.items():
        check(name in symbols, f"missing symbol: {name}")
        check((vectors[index] & ~1) == (symbols[name] & ~1),
              f"vector {index} does not point to {name}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("elf", type=Path, help="GNU Arm firmware ELF")
    args = parser.parse_args()
    try:
        check_vectors(args.elf)
    except (OSError, ValueError, subprocess.SubprocessError) as error:
        print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print("PASS: 98 vectors, SRAM stack, reserved entries, Flash range, "
          "Thumb bits and active handler addresses")
    return 0


if __name__ == "__main__":
    sys.exit(main())
