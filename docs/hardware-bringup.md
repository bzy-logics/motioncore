# Hardware bring-up — STM32F411 single axis

**Status: not yet performed.** This is the checklist for Phase 2. Follow it in
order; do not skip the no-load steps.

## Bill of materials (reference)

* WeAct STM32F411CEU6 "Black Pill" (25 MHz HSE)
* ST-Link V2 or DAPLink probe (SWD) — required for `probe-rs` flashing/recovery
* Step/dir driver with active-low ENABLE (A4988, DRV8825, TMC2209 in step/dir mode)
* NEMA 17 stepper, normally-open limit switch to GND
* **Hardware E-stop** that cuts motor supply to the driver, independent of the MCU
* Logic analyser or oscilloscope

## Wiring (must match `boards/stm32f4/src/board.rs`)

| Signal | Pin | Notes |
|---|---|---|
| STEP | PA0 | 3.3 V logic — check driver V_IH |
| DIR | PA1 | |
| EN (active low) | PA2 | **10 kΩ pull-up to driver logic VCC** so reset/hang = disabled |
| ENDSTOP | PB0 | switch to GND, internal pull-up |
| USB | PA11/PA12 | on-board USB-C |

## Procedure

1. Power the board alone. `motioncore firmware build` → `motioncore firmware flash --yes` → `motioncore firmware verify`.
   The reported build hash must match the artifact.
2. With the driver unpowered, check with the analyser: EN high at reset and
   during flashing; EN low only after `enable`.
3. Unplug USB while enabled → EN must go high within the watchdog timeout (500 ms).
4. Stall the firmware (debugger halt) → IWDG reset within ~100 ms → EN high.
5. Panic safe state: build, flash and verify with `--bench bench-panic` (test build only); during a move tie PB1 to GND →
   EN high and STEP stopped within 50 µs, then IWDG reset after ~100 ms.
6. Step timing (logic analyser on STEP, DIR and PB1, on a `--bench
   bench-isr-probe` build: PB1 is high while the `TIM3` interrupt runs):
   - STEP high **and** low time ≥ 2 µs at 1, 5, 10 and 20 kHz, including the
     back-to-back steps of a tick that emits more than one step. Compare with
     the driver datasheet minimums and DIR setup time.
   - Jitter of step edges against the commanded rate (bounded by one tick,
     50 µs at `TICK_HZ` = 20 kHz).
   - Interrupt execution time at the highest step rate in the profile. It must
     stay well below the 50 µs tick period; an overrun makes ticks late.
   - Commanded vs. observed step count for a full move (must match exactly).
   - A tick stall longer than the late tolerance during motion must fault with
     `SegmentLate` (drives disabled), not emit a burst of steps.
   If any limit fails, lower `max_velocity` in the profile; higher rates need
   timer-compare pulse generation instead of tick-based steps.
7. Motor connected, **no load**, current limit low: jog ±1°, home, stop mid-move, E-stop.
8. Record measured limits in `robot-profiles/single-axis-stm32f411.toml`.

Performance limits must be measured, not assumed (spec section 12).
