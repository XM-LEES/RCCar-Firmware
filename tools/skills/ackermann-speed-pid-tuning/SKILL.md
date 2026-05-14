---
name: ackermann-speed-pid-tuning
description: Project-local workflow for tuning STM32 Ackermann speed PI trim on the current Ackermann firmware path. Use when comparing kp, ki, trim_us candidates, running bench or ground speed PID tests, capturing raw UART telemetry, or creating speed PID tuning records.
---

# Ackermann Speed PID Tuning

Project-local tuning skill for STM32 Ackermann speed PI trim.

## Use When

- Tuning speed response on the current Ackermann firmware path.
- Comparing `kp`, `ki`, or `trim_us` candidates.
- Creating a repeatable bench or ground speed PID test record.

Do not use legacy `vx/vy/vz` command scripts as current acceptance evidence.
Current command protocol is UART4 Ackermann `cmd=0x01` with
`speed_mmps + steering_mrad`.

## Inputs

- Config: `tools/skills/ackermann-speed-pid-tuning/config.json`
- Record template: `tools/skills/ackermann-speed-pid-tuning/templates/bench-test.md`
- Firmware defaults: `WHEELTEC_APP/Inc/app_vehicle_config.h`
- Control implementation: `WHEELTEC_APP/servo_basic_control.c`

Current conservative default:

```text
enable=1
kp=20.0
ki=0.0
trim_us=12
```

## Workflow

1. Confirm branch, commit, flashed firmware, battery voltage, active speed cap,
   and whether the vehicle is raised or on ground.
2. Verify RC takeover, brake, emergency stop, and command timeout before motion.
3. Use `config.json` for targets, timing, candidates, and scoring weights.
4. Keep steering at `0 mrad` unless testing coupled steering behavior.
5. Capture raw telemetry under `logs/YYYY-MM-DD-speed-pid-<scope>/`.
6. Write the summary under `docs/test-records/` using the template.
7. Only force-add raw logs when they are curated and referenced by the summary.

## Scoring

Default per-target score:

```text
score = mae * 1.0 + overshoot * 0.4 + std * 0.05
```

Reject a candidate with obvious oscillation, telemetry dropout, or one unstable
target even if its average score is slightly lower.

## Stop Conditions

- Telemetry frame loss or invalid parsing.
- RC takeover, brake, emergency stop, or timeout check fails.
- Speed exceeds the configured safety cap.
- Vehicle moves unexpectedly.
- Battery voltage drops enough to make later candidates incomparable.

## Acceptance

Do not call a parameter set final until it has been tested on the current
Ackermann protocol, with battery voltage recorded, raw telemetry preserved, and
safety behavior documented.
