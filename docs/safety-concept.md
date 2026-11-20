# Safety concept

- **Status:** Accepted for v0.4 ([#78](https://github.com/bzy-logics/motioncore/issues/78)). Update this document in the same pull request as any change to a protection it lists.
- **Related:** [stop, pause, resume and E-stop specification](stop-behaviour.md) (the contract for every stop function), [ADR 0002: HAL](adr/0002-hal-architecture.md), [safety guide](../website/guide/safety.md) (user-facing summary).

## 1. Scope and claims

MotionCore is **not a certified safety system** and none of its software functions is a safety function in the sense of IEC 61508, ISO 13849 or IEC 62061. This document says which protections exist, at which layer, what each one covers, and what it does not. Its purpose is to keep the code, the documentation and the user's expectations aligned.

Three kinds of protection are distinguished, and must never be described as one another:

| Kind | When | Examples | What it can promise |
|---|---|---|---|
| **Offline validation** | Before a move is sent | Limits, IK reach, singularity refusal, collision check | The **planned** motion satisfies a stated model. Nothing about what the machine actually does |
| **Runtime monitoring** | While the machine runs | MCU per-step soft limits, endstops, watchdogs, interlock input | Detection of the listed conditions and a reaction (drives released), within the stated latency, **if the software and MCU are running** |
| **Hardware safety** | Always, independently | E-stop chain removing driver power, guarding, brakes | The only layer that works with the host, the MCU and every piece of software dead. Not provided by MotionCore |

## 2. Hazards

| # | Hazard | Main protections (section) |
|---|---|---|
| H1 | Unexpected start: motion or energised drives at power-up, reset, flashing or fault reset | ENABLE pull-ups (5), safe start, fault latching, reset never resumes (4.2, 4.3) |
| H2 | Motion beyond the working range of an axis | Planning soft limits (3), MCU per-step soft limits and endstops (4.3) |
| H3 | Collision of the robot with itself or the floor | Certified collision check on the capsule model (3) |
| H4 | Collision with people, workpieces, fixtures or anything not modelled | **Hardware only:** guarding, safe speeds, E-stop chain (5). No software protection |
| H5 | Loss of control: host crash, gateway crash, USB loss, firmware hang or panic | Communication watchdog, hardware watchdog, panic safe state (4.3); engine link supervision (4.2) |
| H6 | Excessive speed or step rate | Planning velocity and acceleration limits (3), MCU rate ceiling and segment validation (4.3) |
| H7 | Lost position: missed steps, motion while disabled, external force | Re-homing after any uncontrolled stop (4.2); position confidence (#103). **Missed steps are not detected** (no encoders) |
| H8 | Failure to stop on request | Stop and E-stop with priority delivery (4.2); hardware E-stop chain (5) |
| H9 | Unauthorised or accidental commands | Gateway roles, origin checks, audit log; UI confirmations (4.4) |
| H10 | Axis falling when de-energised (gravity loads) | **Hardware only:** brakes or self-locking mechanics (5) |
| H11 | Electrical damage or undefined pin states (5 V into 3.3 V inputs, floating ENABLE) | Wiring requirements (5) |

## 3. Offline validation (engine, before motion)

Every check here runs on the **plan**, before any segment is sent. A refused command sends nothing.

| Protection | Covers | Limits | Status |
|---|---|---|---|
| Request validation | Finite values; `speed_scale` in (0, 1]; MoveL `speed_mm_s` in (0, 2000]; at most 256 MoveJPath waypoints; known axes | — | software-tested |
| State gating | Motion only in `ready`; supervisor transition table; nothing accepted in `faulted` or `emergency_stopped` | — | software-tested |
| Homing requirement | Absolute moves, MoveJ, MoveJPath and MoveL need every used axis homed. Unhomed axes may only jog relatively at 25 % speed within one travel range of the power-on position | Homing trusts the endstop and the configured offsets | software-tested |
| Soft limits | Every target and every point of a sampled path inside `[min, max]` | Only as good as the configured limits | software-tested (property tests, #23) |
| Velocity and acceleration limits | Trapezoidal and time-scaled profiles keep every joint within `max_velocity` × `speed_scale` and `max_acceleration`; segments respect the MCU step-rate ceiling | Limits are configuration, not measured capability of the motors (missed steps under load are possible) | software-tested (property tests, #23) |
| IK reach and singularities (MoveL) | Unreachable targets and straight paths through singularities are refused | Numerical IK; the refusal threshold is a joint jump, not a manipulability measure (#84) | software-tested |
| Certified collision check | Self-collision between links and with the floor, on a capsule model, with a 2 mm margin **along the whole planned joint path** of every move type, not only at samples (#77). Paths that cannot be certified to within 0.01 mm are refused | Only the modelled geometry: **no model of the cell, tools beyond their capsules, workpieces or people.** Guarantees the plan, not how closely the motors track it. The model is only as accurate as the configured link geometry and radii | software-tested (property tests); not hardware-validated |
| Stop wins over planning | A Stop or E-stop during planning cancels the move; a move whose start state changed while it was planned is refused (#22) | — | software-tested |

**Collision checking is an advisory planning check on a simplified model.** It never replaces guarding, safe speeds, a risk assessment or the independent E-stop chain.

## 4. Runtime monitoring

### 4.1 Detection layers

| Layer | Runs on | Independent of the host? |
|---|---|---|
| MCU firmware | The MCU, from the motion tick interrupt | Yes, as long as the MCU runs; the hardware watchdog covers a hung MCU |
| Engine | Host | No |
| Gateway and UI | Host and browser | No |

### 4.2 Engine

| Protection | Reaction | Limits | Status |
|---|---|---|---|
| MCU link supervision: 100 ms heartbeats; silence for 1 s or a closed stream | `faulted` (communication lost), all axes unhomed; reconnects; reset lands in `disabled` | The MCU's own watchdog (4.3) reacts earlier; this is the host's view | software-tested |
| Handshake | Refuses an MCU with another protocol version, missing capabilities, or a board id that differs from the configuration; warns on a bench build | — | software-tested |
| Fault latching | Any MCU fault, link loss or plugin failure latches `faulted`; all motion discarded, every axis unhomed; Reset fault never resumes motion and lands in `disabled` | — | software-tested |
| Software E-stop | Priority lane to the MCU, ahead of queued segments; works in every state; latches | Host latency is not bounded | software-tested; latency not measured (#104) |
| Stop | Per the [stop specification](stop-behaviour.md); today each axis brakes on its own (path not preserved, #102) | — | software-tested |
| Shutdown | E-stop before the engine exits (Ctrl-C) | Not on a crash or kill: the MCU watchdog covers that | software-tested |
| Flashing | Refused while a reachable engine reports drives that may be energised | — | software-tested |
| **Not present** | Plan conformance (comparing MCU positions with the committed plan, #86); following error (#43, needs encoders); position confidence beyond homed / unhomed (#103) | | — |

### 4.3 MCU firmware

These run on the MCU and do not trust the host. Every one releases all ENABLE lines and latches a fault that only an explicit reset clears.

| Protection | Trigger | Fault | Status |
|---|---|---|---|
| Segment validation | On enqueue: finite values, start and end velocity within `max_rate`, step count consistent with the kinematics (±1.5 steps), start not in the past or before the queue tail, end position within the soft limits | NACK (nothing queued) | software-tested |
| Per-step soft limits | Before every step: the step would leave `[min_steps, max_steps]` | `soft_limit` | software-tested |
| Endstops | Triggered while moving toward the switch (moving away from a known side is allowed) | `limit_hit` | software-tested |
| Communication watchdog | No frame from the host for `watchdog_ms` (default 500 ms) while a drive is enabled | `communication_lost` | software-tested; RP2040 test 11 pending |
| Late segment / stalled tick | A segment starts later than the board's tolerance, or the tick interval stalls during motion: no catch-up burst | `segment_late` | software-tested |
| Queue underrun | The queue empties after a segment not marked `last` | `queue_underrun` | software-tested |
| Step-rate overrun (RP2040) | Step output falls behind | `step_rate_exceeded` | software-tested; RP2040 test 8 pending |
| Homing failure | Switch not found within the search distance | `homing_failed` | software-tested |
| Interlock input (RP2040 GP27) | E-stop chain monitoring contact opens; also blocks Enable and Reset fault while open | `emergency_stop` | software-tested; RP2040 test 13 pending. **STM32F411: not implemented** (#21) |
| Reset rules | Reset refused while the interlock is open, any axis is enabled or moving, or an axis is still on its endstop with no known safe direction | — | software-tested |
| Hardware watchdog | Firmware hang | MCU reset after ~100 ms; ENABLE released (pull-ups hold it) | RP2040 test 12 and STM32 step 4 pending |
| Panic safe state | Firmware panic: drives disabled and step output stopped before halting, then watchdog reset (#68) | link loss seen by the engine | build tested in CI; measurement pending (#68) |

On RP2040, endstop and fault reactions can trail the commanded position by one 200 µs step window.

### 4.4 Gateway and UI

Operational protections against mistakes and misuse, not safety functions:

- **ratavyr:** roles (viewers watch and stop; operators move; administrators flash); origin and DNS-rebinding checks; refuses to start unauthenticated on a non-loopback address; brute-force blocking; rate limits with `stop` and `estop` exempt; audit log of every command.
- **sindrava:** E-STOP on every screen, never disabled, confirmed only by telemetry; confirmations before Enable, Home and Reset fault; motion controls disabled on stale telemetry (> 1.5 s); flashing requires confirming the drives are unpowered.

## 5. Hardware safety (provided by the integrator, not by MotionCore)

| Requirement | Why |
|---|---|
| **Independent E-stop chain** that removes driver power through a contactor or safety relay, without passing through any software (#21 defines the requirements) | The only protection that works with the host, MCU and software dead (H4, H5, H8) |
| Interlock monitoring contact to GP27 on RP2040 | Lets the firmware latch the E-stop and refuse Enable; supplements, never replaces, the chain |
| **External pull-up on every ENABLE line** (e.g. 10 kΩ to the driver's logic supply) | Drivers stay disabled during reset, bootloader, watchdog reset, panic and with no firmware (H1, H5) |
| 3.3 V-compatible driver inputs or level shifters | Both boards are not 5 V tolerant (H11) |
| Guarding, safe speeds and a risk assessment for the actual cell | Nothing in software models people or the environment (H4) |
| Brakes or self-locking mechanics on gravity-loaded axes | Disable and every fault release holding torque (H10) |
| Safe Torque Off only with drivers that certify it | Releasing a driver's ENABLE input is not STO |

## 6. Validation status

Nothing in this document is **hardware-validated** yet. The RP2040 bench session ([`validation/rp2040-bench-session.md`](validation/rp2040-bench-session.md), tests 1–16) and the STM32F411 bring-up procedure are the first step; stop latencies follow the metrics of the [stop specification](stop-behaviour.md) section 8 (#104).

Validation levels: *implemented* (code exists), *simulated* (runs on the virtual MCU), *software-tested* (automated tests in CI), *hardware-validated* (measured on physical hardware, with the board, build hash and setup recorded).

## 7. Rules for documentation

- Never describe offline validation as runtime protection, or software protection as a safety function.
- Never write that a collision check makes motion safe; say what it checks (the planned path against the capsule model).
- Give each protection's validation level; do not call anything hardware-validated without recorded measurements.
- A protection that is planned but not implemented is listed as such, with its issue.
