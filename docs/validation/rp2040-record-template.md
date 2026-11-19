# RP2040 validation record: YYYY-MM-DD

Copy this file to `docs/validation/rp2040-YYYY-MM-DD.md` and fill it in. Procedure: [`rp2040-bench-session.md`](rp2040-bench-session.md); test definitions and pass criteria: [`docs/rp2040.md`](../rp2040.md) §9.

## Setup

| Item | Value |
|---|---|
| Date(s) | |
| Operator | |
| Board | Raspberry Pi Pico, revision: |
| Drivers | model, or "none" (motor supply **off**) |
| ENABLE pull-ups | value, to which rail |
| Logic analyser | model, sample rate |
| Host | OS, USB port |
| Engine | `motioncore info` output |
| Profile | `robot-profiles/six-axis-rp2040-pico-timing.toml` @ commit |
| Build A (release) hash | |
| Build B (tick-ISR probe) hash + description of the local edit | |
| Build C (bench-panic) hash | |
| Captures | link to the zip on the issue |

## Results

Result: **pass**, **fail** (link the new issue) or **not run** (why).

| # | Test | Build | Measured | Pass criterion (`rp2040.md` §9) | Result | Capture |
|---|---|---|---|---|---|---|
| 1 | Safe start | A | ENABLE level: no firmware / BOOTSEL / flashing / running | All ENABLE lines high throughout | | |
| 2 | Identity | A | board, build hash from `firmware verify` | `rp2040-pico`, hash matches | | |
| 3 | STEP high time | A | at 1 / 10 / 50 kHz: min / max µs | 2.0 µs ± 0.1 µs | | |
| 4 | STEP low time | A | shortest low phase at 1 / 10 / 50 kHz, µs | ≥ 2.0 µs | | |
| 5 | DIR setup / hold | A | DIR edge → next STEP rise; last STEP fall → DIR edge, µs | ≥ 2.0 / ≥ 2.2 µs | | |
| 6 | Frequency accuracy | A | edges in 1 s at 1 / 10 / 50 kHz; total steps vs commanded | within 0.1 %; exact total | | |
| 7 | Jitter | B | STEP period histogram at constant velocity: min, max, spread | spread ≤ one 200 µs window effect; no period below minimum | | |
| 8 | Max sustainable rate | A | highest rate without `StepRateExceeded` (≤ 50 kHz via engine, see session §5) | record; advertised ≤ 50 % of it | | |
| 9 | Six axes simultaneously | B | six channels at 50 kHz: fault? max ISR time (GP28 high) µs | no fault; ISR < 100 µs | | |
| 10 | Stop latency | A | `estop`: frame receipt → last STEP edge; → ENABLE high, ms | both within 1 ms | | |
| 11 | Watchdog | A | USB unplug → ENABLE high, ms | ≤ 500 ms | | |
| 12 | Hang | A | core 0 halted → reset, ms; ENABLE level | reset ≈ 100 ms; ENABLE high | | |
| 13 | Interlock | A | GP27 open while moving: fault, ENABLE, reset refused until closed | `EmergencyStop`, drives off, reset refused | | |
| 14 | Endstop | A | per axis: overtravel in steps at homing speed | trigger detected; ≤ 1 step | | |
| 15 | Coordinated-move skew | A | `tools/step-skew` output: offsets, skew µs, jitter µs / steps (full speed, `--speed 0.25`, max rate) | skew ≤ 50 µs, constant; edges within one window + 1 step (tool defaults) | | |
| 16 | Panic safe state | C | GP28 to GND during a six-axis move: trigger → ENABLE high, µs; last STEP edge after the trigger, µs; reset after, ms; engine fault | ENABLE high, no STEP within 10 µs; reset ≈ 100 ms; engine faults | | |

## Test 15 output

```text
(paste the step_skew.py output for each run)
```

## Findings and follow-ups

- (anything unexpected, with the issue it was filed as)

## Sign-off

- [ ] Every row has a result and a capture.
- [ ] Failures are filed and linked.
- [ ] `rp2040.md` §9 summary updated; issues moved to Done; changelog validation levels updated for what was measured.
