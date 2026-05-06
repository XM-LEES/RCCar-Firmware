# ESC PWM Bench Mapping

## Test Context

- Date: 2026-05-06
- Branch: `calibration/esc-pwm-cutoff-sweep`
- Commit: `4ce739a`
- Firmware mode: `APP_ESC_PWM_SWEEP_MODE = 1`
- UART command meaning: raw `vx` is ESC PWM offset in microseconds.
- ESC center: `1500 us`
- Sweep step: `1 us`
- Hold time per point: `5 s`
- Steady window: last `3 s`
- Speed limiter: trigger at `5.0 m/s`, release at `4.5 m/s`
- Round 1 summary: `docs/investigations/esc-pwm-bench-mapping-raw.csv`
- Round 2 summary: `docs/investigations/esc-pwm-bench-mapping-round2-summary.csv`
- Round 2 raw samples: `docs/investigations/esc-pwm-bench-mapping-round2-samples.csv`

This is a bench/no-load mapping. It should not be treated as the final ground-load mapping.
Round 2 is the preferred dataset because it records every UART telemetry frame
and uses a longer inter-point neutral wait.

## Round 2 Rest Policy

After each PWM point, the firmware was commanded back to neutral. The script
waited at least `2 s` and continued until several consecutive samples were
near zero speed, up to `6 s` maximum.

In this run, all points reached zero before the maximum wait:

| Metric | Value |
|---|---:|
| Points | `217` |
| Raw telemetry samples | `30791` |
| Average neutral wait | `2.04 s` |
| Maximum neutral wait | `2.42 s` |
| Last recorded rest speed | `0 m/s` for all points |

## Summary

| Direction | Dead / unreliable range | Stable start | Last usable before limiter | Limiter-near range |
|---|---:|---:|---:|---:|
| Forward | `1502..1545 us` | `1546 us` | `1603 us` | `1604..1606 us` |
| Reverse | `1498..1446 us` mostly dead/unreliable | `1445 us` weak, `1444 us` reliable | `1390 us` | `1389..1387 us` |

Recommended conservative bench ranges:

```c
#define APP_ORIN_ESC_FORWARD_START_US 1546U
#define APP_ORIN_ESC_FORWARD_MAX_US   1603U

#define APP_ORIN_ESC_REVERSE_START_US 1444U
#define APP_ORIN_ESC_REVERSE_MAX_US   1390U
```

`1445 us` can move in reverse at about `-0.08 m/s`, but it is weak and has zero-valued samples. `1444 us` is a better reliable reverse start point.

## Target-Speed Lookup

Nearest round-2 measured PWM points below the limiter:

| Target speed | Forward PWM / measured | Reverse PWM / measured |
|---:|---:|---:|
| `0.1 m/s` | `1547 us` / `0.127 m/s` | `1445 us` / `-0.080 m/s` |
| `0.2 m/s` | `1548 us` / `0.192 m/s` | `1444 us` / `-0.179 m/s` |
| `0.3 m/s` | `1550 us` / `0.293 m/s` | `1442 us` / `-0.287 m/s` |
| `0.5 m/s` | `1554 us` / `0.534 m/s` | `1438 us` / `-0.526 m/s` |
| `0.8 m/s` | `1559 us` / `0.809 m/s` | `1433 us` / `-0.817 m/s` |
| `1.0 m/s` | `1562 us` / `1.021 m/s` | `1430 us` / `-1.006 m/s` |
| `1.5 m/s` | `1568 us` / `1.462 m/s` | `1423 us` / `-1.513 m/s` |
| `2.0 m/s` | `1574 us` / `1.979 m/s` | `1417 us` / `-2.033 m/s` |
| `2.5 m/s` | `1580 us` / `2.514 m/s` | `1412 us` / `-2.491 m/s` |
| `3.0 m/s` | `1585 us` / `2.974 m/s` | `1407 us` / `-2.967 m/s` |
| `3.5 m/s` | `1590 us` / `3.464 m/s` | `1402 us` / `-3.463 m/s` |
| `4.0 m/s` | `1596 us` / `4.038 m/s` | `1397 us` / `-3.955 m/s` |
| `4.5 m/s` | `1601 us` / `4.469 m/s` | `1391 us` / `-4.500 m/s` |

## Notes For Open-Loop Mapping

- The old `1560 us` forward start was too high for low-speed control; on the bench it gives about `0.9 m/s` in round 2.
- The measured curve is not perfectly linear. A simple `start -> max` linear map will be acceptable for a first open-loop baseline, but a lookup or piecewise map will track commanded speed better.
- For a first normal firmware update, keep the 5 m/s limiter enabled and avoid mapping commands above about `4.5 m/s`.
- Ground testing should produce a separate mapping document because load, tire contact, battery voltage, and slip will change the curve.
