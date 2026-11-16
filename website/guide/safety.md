# Safety model

::: danger Read this before connecting a motor
MotionCore is **not a certified safety system**. It is designed to be safety-conscious, but a software stop can fail for reasons software cannot see: a crashed host, a hung USB stack, a damaged cable, a stuck driver. Every physical setup **must** have an independent hardware E-stop circuit that removes driver power.
:::

## Principles

> Plugin-first, but not plugin-controlled safety. Firmware-buildable, but not arbitrary-hardware magic. Hardware-independent at the interface, but hardware-validated in practice.

1. **Defence in depth.** Limits are checked by the engine when planning *and* by the MCU before every step. Neither trusts the other.
2. **Fail to disabled.** Loss of communication, a stalled firmware, a late segment, an empty queue mid-move: each ends with drives de-energised, not with a guess.
3. **Faults latch.** A fault stays active until explicitly reset, and a reset never resumes motion. Axes must be re-homed afterwards.
4. **Plugins propose, the core decides.** Anything motion-related from a plugin passes the planner's validation and the MCU's own checks before a single step is emitted.
5. **Hardware safety is independent.** The hardware E-stop must work with Linux, the engine, the gateway and every plugin dead.

## What each layer enforces

### MCU firmware

| Protection | Behaviour |
|---|---|
| Communication watchdog | No frame from the host for `watchdog_ms` (default 500 ms) → all drives disabled, queue flushed, `communication_lost` latched |
| Soft limits | Checked before every step; a step that would leave `[min, max]` faults with `soft_limit` |
| Endstops | Triggered while moving toward the switch → `limit_hit`, no further step |
| Step-rate ceiling | The firmware never exceeds the axis `max_rate`. On RP2040, falling behind faults with `StepRateExceeded` rather than letting lag grow |
| Queue underrun | Queue runs dry after a segment that was not marked `last` → `queue_underrun`, drives disabled |
| Late segment | Segment arrives after its start time (beyond tolerance) or the tick stalls during motion → `segment_late`, drives disabled. No catch-up burst of steps |
| Segment validation | Every segment is re-validated; anything wrong is NACKed |
| Hardware watchdog | Firmware hang → independent watchdog reset (~100 ms) → ENABLE released |
| Interlock input (RP2040, GP27) | Open → E-stop latched; `Enable` and `ResetFault` refused until closed |

### Engine

- Explicit state machine; any transition not in the table is rejected.
- Plans validated for finite values, limits, velocity and acceleration before anything is sent.
- Unhomed axes may only jog relatively, at 25 % speed, within one travel range of the power-on position.
- Detects a silent MCU (1 s) or closed stream → `faulted`, reconnects; reset lands in `disabled`.
- Refuses an MCU with the wrong protocol version, missing capabilities, or a board id that differs from the configuration.
- Stop and E-stop are submitted under the engine lock, so a stop is never overtaken by a stale segment.
- On shutdown (Ctrl-C), issues an E-stop before exiting.
- Refuses to flash while drives may be energised.

### Gateway

- Roles: viewers can watch and stop, operators can move, only admins can flash.
- `stop` and `estop` are never rate-limited.
- Every command is audited with role, client IP, parameters, outcome and duration.
- Refuses to start on a non-loopback address with authentication off.

Details: [ratavyr: deployment](https://ratavyr.bzy-logics.com/guide/deployment).

### UI

- E-STOP in the header on every screen, never disabled.
- Confirmation dialogs before Enable, Home and Reset fault (Cancel has focus).
- Motion controls disabled unless telemetry is live and fresh (< 1.5 s old).
- Flash requires an explicit "drives are powered off" confirmation.

Details: [sindrava: safety behaviour](https://sindrava.bzy-logics.com/guide/safety).

## Electrical requirements

- **ENABLE needs an external pull-up** to the driver's logic supply (e.g. 10 kΩ), so drivers stay disabled during reset, bootloader mode, a watchdog reset, or with no firmware at all. ENABLE is active-low.
- Both supported boards use **3.3 V logic and are not 5 V tolerant.** Use drivers whose inputs accept 3.3 V, or level shifters.
- The MCU never powers motors. Use external step/dir drivers.
- Wire the E-stop to **remove driver power** (contactor or safety relay). On RP2040, also wire an auxiliary contact to the GP27 interlock so the firmware knows.

## Before the first move

Follow the bring-up procedure for your board, in order, without skipping the no-load steps:

- [STM32F411 bring-up](/hardware/stm32f411#bring-up-procedure)
- [RP2040 validation](/hardware/rp2040#hardware-validation)

Performance limits must be **measured** on your hardware, not assumed.

## Known gaps

- Joint limits do not guarantee collision-free motion; there is no geometry or environment model yet.
- "Measured" position is the MCU step counter. There is no encoder feedback, so missed steps are not detected.
- On RP2040, endstop and fault reaction can trail the commanded position by one 200 µs window.
