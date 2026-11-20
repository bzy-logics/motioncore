# ADR 0002: Hardware abstraction layer architecture

- **Status:** Accepted (2026-11-18)
- **Issue:** [#67](https://github.com/bzy-logics/motioncore/issues/67) · Epic [#75](https://github.com/bzy-logics/motioncore/issues/75)
- **Applies to:** `crates/firmware-hal`, `crates/firmware-stepper`, `crates/firmware-core`, `crates/firmware-pio-step`, `crates/simulator`, `boards/*`
- **Related:** [safety concept](../safety-concept.md) (which protections the firmware and HAL provide, and which only hardware does)

## Context

The HAL is the boundary between MotionCore's hardware-independent firmware
runtime and each microcontroller. It has to support several MCU families
(RP2040, STM32F411 today), allow hardware-specific optimisations (RP2040 PIO,
STM32 timers/DMA), run the same code in simulation, and keep pulse generation
deterministic regardless of host or network load. It must not contain
kinematics, trajectory planning or high-level motion logic: those belong to
the engine.

### Current state

The layering already separates hardware from logic:

```text
engine (host) ── segments over the MCU protocol ──▶ firmware-core (dispatch, watchdog, faults)
                                                      └─ firmware-stepper (segments → steps per tick)
                                                           └─ Hal + Clock (crates/firmware-hal)
                                                                ├─ boards/rp2040   (+ firmware-pio-step)
                                                                ├─ boards/stm32f4
                                                                ├─ crates/simulator (virtual MCU)
                                                                └─ firmware-core tests (second sim HAL)
```

- `Hal` is one trait: `step(axis, forward)` (one pulse), `set_enable`,
  `endstop_raw`, `interlock_ok` (default `true`), `flush_pulses` (default
  `true`); `Clock::now_us` is separate.
- **RP2040** (125 MHz, 6 axes): the tick interrupt runs every 200 µs; `step()`
  only buffers, and `flush_pulses()` hands each axis' steps for the next
  200 µs window to a PIO state machine, which spaces them evenly with exact
  pulse timing. A GP27 interlock input is implemented. Hardware watchdog
  100 ms.
- **STM32F411** (96 MHz, 1 axis): the TIM3 interrupt runs at 20 kHz (50 µs);
  `step()` bit-bangs each pulse inside the interrupt with busy-wait delays
  (2 µs DIR setup, 2 µs high, 2 µs low — about 6 µs per step). Pulses are
  therefore quantised to tick boundaries, arrive in bursts, and interrupt
  time grows with the step count, which rules out multi-axis operation and
  hardware-timed output on this interface. No interlock input: the default
  `interlock_ok() = true` reports "safe". IWDG 100 ms.
- Motion runs in the timer interrupt; USB is polled in the main loop. Real-time
  pulse generation is already independent of host communication.
- Both boards use `panic_halt`: on a panic, outputs freeze in their current
  state (drives stay enabled; queued PIO pulses keep running) until the
  watchdog resets the MCU.
- Two simulation HALs exist (`crates/simulator` and a test HAL in
  `firmware-core`), plus a PIO emulator in `firmware-pio-step`.
- Encoder, digital I/O, PWM and analog capabilities exist only as capability
  bits in the protocol; nothing implements them.

## Decision

### 1. Keep the crate layout; evolve the contracts

No new `motioncore-hal-*` crates for now. The existing crates already provide
the proposed boundaries:

| Role | Crate |
|---|---|
| Common traits and contracts | `crates/firmware-hal` |
| RP2040 implementation | `boards/rp2040` with `crates/firmware-pio-step` (PIO program and host emulator) |
| STM32 implementation | `boards/stm32f4` |
| Simulation implementation | `crates/simulator` (one simulation HAL; the `firmware-core` test HAL is removed) |
| Hardware-independent execution | `crates/firmware-stepper`, `crates/firmware-core` (not part of the HAL) |

Board drivers move into library crates (`hal-rp2040`, `hal-stm32`) only when a
second board of the same family needs them; renaming alone adds churn without
benefit.

### 2. Capability traits, nothing implicit

The single `Hal` trait is split into small capability traits.

- **Required** for every board: `StepGen`, `DriverEnable`, `Endstops`,
  `Interlock`, `MonotonicClock`, `Watchdog`.
- **Optional**: `EncoderInput`, `DigitalIo`, `Pwm`, `AnalogIn`, reached as
  `Option<&mut dyn Trait>` so the runtime can tell at run time whether a board
  has them.
- No default method may pretend a capability exists. A board without an
  interlock input implements `Interlock` explicitly as *absent*, which the
  board reports in its capabilities; the engine's policy decides whether that
  board may enable drives (see #21).
- The capability bits in `Identity` are derived from what a board implements,
  not maintained by hand.

### 3. Window-level step generation (`StepGen`)

The hardware-independent runtime (`firmware-stepper`) computes, for each tick
window, every axis' step count, direction, first-step offset and spacing, and
passes the window to the backend:

```rust
pub trait StepGen {
    /// Hardware-timed pulses for the next window. Errors when the hardware is
    /// still busy with an earlier window or the rate exceeds pulse timing.
    fn schedule(&mut self, window: &StepWindow) -> Result<(), StepError>;
}
```

- **RP2040:** the PIO path, unchanged in behaviour. The refactor is verified
  with a golden test: the PIO command stream for a set of recorded moves must
  be bit-identical before and after (using `firmware-pio-step`'s emulator), so
  hardware measurements taken before the refactor stay valid.
- **STM32:** a timer/DMA backend (#46) replaces bit-banging; an interim backend
  keeps today's behaviour until then.
- **Simulation:** records exact edge timestamps for timing tests.
- Pulse width, spacing and DIR setup/hold are the backend's responsibility and
  are documented per backend (#72).

The engine-to-MCU protocol (timestamped constant-acceleration segments) does
not change.

### 4. Initialisation, ownership, errors, safe shutdown

- **Initialisation order:** clocks → watchdog → safe output levels (drives
  disabled, STEP low) → capability setup → interrupts unmasked → host link.
  An unprogrammed, resetting or faulted MCU always leaves drives disabled.
- **Ownership:** each backend owns its pins and peripherals from construction;
  no global access except the panic path.
- **Errors:** one error type per capability (for example `StepError::Busy`,
  `StepError::RateExceeded`); the runtime maps them to protocol fault codes.
- **Safe shutdown:** `safe_shutdown()` is infallible and callable from the
  panic handler and every fault path. It drives every ENABLE output to its
  disabled level, stops and clears pulse hardware (PIO state machines and
  FIFOs, timers and DMA), and puts other outputs in their configured safe
  state. It does not replace the external E-stop chain, which must remove
  driver power independently of the firmware. Each board installs a panic
  handler that calls it, then waits for the watchdog (#68).

### 5. Buses, PWM and analog input

- UART, SPI, I2C and CAN use the embedded-hal 1.0 and embedded-can traits
  that `rp2040-hal` and `stm32f4xx-hal` already implement. Device drivers
  (for example TMC2209 configuration over UART, CAN servo drives) are built
  above the HAL on those traits (#73). MotionCore defines no bus traits of its
  own.
- `Pwm` and `AnalogIn` are specified now and implemented together with their
  first consumer (an RC-servo gripper, an analog sensor, driver temperature or
  current monitoring) (#74).

### 6. Timing guarantees are documented per backend

Every backend documents its step timing resolution, jitter bound, pulse
timing, maximum rates, tick period, late tolerance, worst-case interrupt time
and stop latency, each marked as a design value or a measured value with the
firmware build hash (#72).

## Consequences

- The STM32 can gain multi-axis, hardware-timed output without changing the
  runtime above the HAL.
- Unsupported hardware is visible to the engine and the user instead of being
  silently assumed.
- Panics no longer leave drives energised for up to a watchdog period.
- One simulation backend serves the virtual MCU and the firmware tests, and
  can verify per-step timing against the plan.
- Cost: a refactor of `firmware-stepper`, both boards and the simulator. The
  RP2040 golden test keeps its validated behaviour unchanged.

## Alternatives considered

- **Separate `motioncore-hal-*` crates now.** Rejected for now: same boundaries
  as today's crates with added churn; revisit when a second board of one
  family appears.
- **Keep the per-pulse `step()` interface.** Rejected: it quantises STM32
  pulses to the tick, puts busy-waits in the interrupt and prevents
  timer/DMA output.
- **MotionCore-specific bus traits.** Rejected: embedded-hal 1.0 is the
  ecosystem standard and is already implemented by the board HAL crates.
- **Default methods for optional hardware.** Rejected: defaults such as
  `interlock_ok() = true` hide missing safety functions.

## Follow-up work

| Issue | Milestone | Work |
|---|---|---|
| [#68](https://github.com/bzy-logics/motioncore/issues/68) | v0.4 | Safe state on firmware panic |
| [#69](https://github.com/bzy-logics/motioncore/issues/69) | v0.4 | Safe behaviour on MCU reset, power and communication loss |
| [#70](https://github.com/bzy-logics/motioncore/issues/70) | v0.5 | `StepGen` and required capability traits, RP2040 golden test |
| [#40](https://github.com/bzy-logics/motioncore/issues/40) | v0.5 | Optional capability traits and negotiation |
| [#71](https://github.com/bzy-logics/motioncore/issues/71) | v0.5 | One simulation HAL with exact timestamps and fault injection |
| [#72](https://github.com/bzy-logics/motioncore/issues/72) | v0.5 | Timing guarantees per backend |
| [#46](https://github.com/bzy-logics/motioncore/issues/46) | v0.7 | STM32 timer/DMA step generation, multi-axis |
| [#73](https://github.com/bzy-logics/motioncore/issues/73) | v0.7 | Bus and device-driver layer on embedded-hal 1.0 |
| [#74](https://github.com/bzy-logics/motioncore/issues/74) | v0.7 | PWM and analog input, on demand |
