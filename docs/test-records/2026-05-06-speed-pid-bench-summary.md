# 2026-05-06 Speed PID Bench Summary

## Source

- Source branch: `origin/feature/orin-speed-pid`
- Source commit: `989e445 Add serial PID tuning workflow and bench logs`
- Current migration target: `feature/ackermann-chassis`
- Test scene: wheels raised on bench
- Serial link: UART4, 115200 baud
- Motion command source during original test: legacy `vx/vy/vz` command frame from `tools/pid_auto_tune.ps1`

The current `feature/ackermann-chassis` branch uses the newer `cmd=0x01`
Ackermann command (`speed_mmps + steering_mrad`). These records are preserved as
bench evidence for the speed PI trim values, but they must be revalidated with
the current Ackermann command protocol before final vehicle acceptance.

## Curated Raw Logs

Curated logs are stored under:

```text
logs/2026-05-06-speed-pid-bench/
```

Files:

- `raw-p18-p20-p22-low-mid.csv`
- `summary-p18-p20-p22-low-mid.csv`
- `recommended-p18-p20-p22-low-mid.txt`
- `raw-p20-trim-sweep.csv`
- `summary-p20-trim-sweep.csv`
- `recommended-p20-trim-sweep.txt`

The full uncurated log set remains available in `origin/feature/orin-speed-pid`.

## Main Candidate Sweep

Targets:

```text
-2.5, -1.8, -1.2, -0.8, -0.5, -0.3,
 0.3,  0.5,  0.8,  1.2,  1.8,  2.5 m/s
```

Compared candidates:

| Candidate | Kp | Ki | Trim us | Mean score | Notes |
| --- | ---: | ---: | ---: | ---: | --- |
| `p18_i0_t12` | 18.0 | 0.0 | 12 | not selected | More steady than high-gain candidates, but larger positive high-speed error |
| `p20_i0_t12` | 20.0 | 0.0 | 12 | 0.0558 | Best robust candidate in this sweep |
| `p22_i0_t12` | 22.0 | 0.0 | 12 | not selected | Slightly better at some high-speed points, worse low-speed variation |

Script recommendation:

```text
recommended_set=p20_i0_t12
enable=1
kp=20.0
ki=0.0
trim_us=12
mean_score=0.0558
```

## Trim Sweep

Trim sweep held `kp=20.0`, `ki=0.0` and compared trim limits `10/12/14/16 us`.

Script recommendation:

```text
recommended_set=p20_i0_t14
enable=1
kp=20.0
ki=0.0
trim_us=14
mean_score=0.0553
```

Engineering decision for firmware defaults:

```text
enable=1
kp=20.0
ki=0.0
trim_us=12
```

Rationale: `trim_us=14` had a slightly lower aggregate score, but the same run
showed a clear unstable point around `-1.8 m/s` and was collected near the end
of the battery session. `trim_us=12` is retained as the safer default until a
fresh-battery Ackermann-protocol retest confirms the higher trim limit.

## Risks And Follow-Up

- Original test used the legacy command frame, not the current Ackermann
  command frame.
- Original test was bench-only with wheels raised.
- Battery state changed during the session and likely affected later sweeps.
- Current `feature/ackermann-chassis` reset-time speed caps are much lower than
  the `+-2.5 m/s` sweep range, so acceptance testing must state the active cap.
- Next test should repeat `kp=20, ki=0, trim=12` and `trim=14` on the current
  Ackermann protocol with a fresh battery.
