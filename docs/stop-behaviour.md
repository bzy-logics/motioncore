# Stop, pause, resume and emergency stop

- **Status:** Draft for owner review ([#20](https://github.com/bzy-logics/motioncore/issues/20)). Sections 1–5 describe the current behaviour; section 6 specifies pause and resume, which are not implemented yet ([#29](https://github.com/bzy-logics/motioncore/issues/29)); section 9 lists the decisions needed before #29 starts.
- **Applies to:** engine (`crates/core`, `crates/safety`), firmware (`crates/firmware-core`, `crates/firmware-stepper`, `boards/*`), ratavyr, sindrava, MCL programs.

Every stop-like function in MotionCore follows this document. The firmware, the engine, the gateway, the UI and MCL programs must implement the same semantics.

## 1. Principles

1. **The emergency stop is hardware.** The physical E-stop chain removes driver power independently of the MCU, the host, the gateway and the browser. Everything in software described here is a *convenience or a second layer*; none of it is a safety function in the sense of a certified safety-related control system.
2. **Software E-stop has precedence** over every other command and state, and latches.
3. **Nothing resumes by itself.** Resetting a fault or an E-stop never restarts motion; a stopped or faulted move is never continued automatically. Only an explicit Resume of an explicitly paused move (section 6) continues motion, and only if nothing has changed since.
4. **Positions after an uncontrolled stop are not trusted.** After an E-stop or a fault, every axis must be re-homed.
5. **Stop always wins over a move being planned** (#22): a Stop or E-stop that arrives while a move is being planned cancels it; nothing is sent.

## 2. Definitions

| Function | Purpose | Drives | Path | Positions afterwards |
|---|---|---|---|---|
| **E-stop (hardware)** | Remove motion energy in an emergency | Power removed by the safety circuit | Not controlled | Lost (re-home) |
| **E-stop (software)** | Immediate halt from the UI, API or CLI | ENABLE released at once; step output halted | Not controlled | Lost (re-home) |
| **Stop** | End the current motion in a controlled way | Stay enabled | Each axis decelerates on its own (today) — **may leave the planned path** | Kept (homed axes stay homed) |
| **Pause** (*not implemented*) | Hold the current motion so it can be resumed | Stay enabled | Decelerates **along the planned path** | Kept |
| **Resume** (*not implemented*) | Continue a paused motion to its original target | Stay enabled | Continues the same path | Kept |
| **Disable** | Controlled stop, then de-energise | Released after the stop | As Stop | Kept, but holding torque is gone (gravity-loaded axes may move) |
| **Fault** | Reaction to a detected error | Released at once (MCU) | Not controlled | Lost (re-home) |
| **Reset fault** | Acknowledge a fault or E-stop | Stay disabled | — | Still lost (re-home) |

Mapping to IEC 60204-1 stop categories, as a guide only: hardware E-stop ≈ category 0; Stop ≈ category 2; Disable ≈ category 1. Releasing a driver's ENABLE input is not Safe Torque Off unless the driver certifies it.

## 3. Current behaviour

### Software E-stop

- **Engine:** in any state, even without an MCU link: discards all planned and queued motion, marks every axis unhomed and disabled, latches `emergency_stopped`, then sends `EmergencyStop` to the MCU on the priority lane (ahead of queued segments).
- **MCU:** halts step generation and releases every ENABLE at once; latches `EmergencyStop`.
- **RP2040:** the GP27 interlock input (E-stop chain monitoring contact) trips the same latch when it opens, and blocks Enable and fault reset while open. **The STM32F411 has no interlock input yet** (#21).
- **Status:** implemented and software-tested; stop latency not yet measured on hardware (RP2040 test 10).

### Stop

- **Engine:** discards planned motion that has not been sent (`clear_motion`), cancels a home-all run, and sends `Stop` to the MCU. The robot stays `running` until fresh MCU status shows every axis at rest with an empty queue, then returns to `ready`; commanded positions are resynced from the MCU step counters. If homing was in progress, the axis stays unhomed.
- **MCU:** clears every axis' segment queue and aborts homing; each moving axis decelerates from its current velocity at its own `stop_decel` (profile `stop_deceleration`, default 2 × `max_acceleration`). Drives stay enabled.
- **Consequence:** axes with different velocities and decelerations stop at different times, so **a coordinated (MoveJ, MoveJPath) or Cartesian (MoveL) move generally leaves its planned path while stopping.** The deviation is bounded by each axis' braking distance `v² / (2 · stop_decel)`.
- **Status:** implemented and software-tested; not measured on hardware.

### Disable

Controlled Stop as above; once the MCU reports every axis idle (timeout 3 s, then fault), every ENABLE is released and the state becomes `disabled`. Positions stay known.

### Faults

Any fault (MCU communication watchdog, queue underrun, late segment, endstop hit while moving towards it, soft limit, step-rate overrun, homing failure, plugin failure, link loss) latches `faulted`: the MCU releases every ENABLE immediately, the engine discards all motion and marks every axis unhomed. Fault reset is refused while the hazard persists (for example an endstop still pressed with no known safe direction, or the interlock open) and always lands in `disabled`.

### Firmware panic

Drives are disabled and pulse output stopped immediately, then the hardware watchdog resets the MCU (#68, pending hardware measurement). The engine sees the link loss as a fault.

## 4. State × action

| State | Stop | Disable | E-stop | Reset fault |
|---|---|---|---|---|
| disconnected | refused (no MCU) | refused | latches `emergency_stopped` | refused |
| initializing | accepted, no effect | refused | latches | refused |
| disabled | accepted, no effect | no change | latches | refused |
| ready | accepted; cancels a move being planned | → disabled | latches | refused |
| homing | homing aborted; axis unhomed; → ready when idle | stop, then → disabled | latches | refused |
| running | controlled stop (section 3); → ready when idle | stop, then → disabled | latches | refused |
| faulted | refused by the MCU (nothing moving) | no change | latches `emergency_stopped` | → disabled if the hazard is gone |
| emergency_stopped | refused by the MCU | no change | stays latched | → disabled if the hazard is gone |

Motion commands (jog, move, home, MoveJ, MoveJPath, MoveL) are accepted only in `ready`; while a Stop is in progress the state is `running`, so they are refused until the robot is at rest.

## 5. Latency and distance targets

| Function | Target | Measured |
|---|---|---|
| Software E-stop | ENABLE released and no further STEP edge within **1 ms** of the frame reaching the MCU | Not yet (RP2040 test 10) |
| Interlock opening | Same as software E-stop, from the input edge | Not yet (RP2040 test 13) |
| Firmware panic | ENABLE released within 10 µs (RP2040) / 50 µs (STM32) | Not yet (#68) |
| Stop | Deceleration starts within one motion tick of the frame reaching the MCU (200 µs RP2040, 50 µs STM32); each axis stops within `v² / (2 · stop_decel)` | Not yet |
| Communication loss | Drives released within the profile's `watchdog_ms` | Not yet (RP2040 test 11) |

Host-side latency (UI → gateway → engine → USB) is not bounded and must not be relied on for safety.

## 6. Pause and resume (specification for #29)

**Pause** is a feed hold: the remaining trajectory is re-timed so that the path parameter decelerates to zero while every axis stays on the planned path, then the motion holds with drives enabled.

- Allowed in `running` (MoveJ, MoveJPath, MoveL, jog and MoveTo); not in `homing` (see 9.2).
- Deceleration: as fast as every axis' `max_acceleration` (or `stop_decel`, see 9.1) allows along the path.
- State: a `paused` state (or a flag on `running`), reported in telemetry with the paused motion's id (#28).
- Stop while paused discards the paused motion (→ `ready`). E-stop, fault and Disable discard it permanently.

**Resume** continues the paused motion from the held point to its original target, along the original path, with the original speed limits.

- Allowed only from `paused`, and only if the engine's state revision has not changed since the pause (no other command, no fault, no disable, no MCU reconnection, no manual motion).
- The remaining path is validated again (limits, collision check) before anything is sent.
- Refused with `invalid_state` otherwise; the operator then issues a new move.

**MCL programs:** Pause pauses the program at the current motion instruction; Resume continues it. Stop ends the program. A fault or E-stop ends the program and reports the instruction and source line.

## 7. Interfaces

- **Engine API:** `Stop`, `EmergencyStop`, `Disable`, `ResetFault` exist. `Pause` and `Resume` will be added with #29 (additive; version per the versioning rules).
- **ratavyr:** `stop` and `estop` are allowed for the viewer role (anyone who can see the robot can stop it); everything else needs the operator role. Pause should be allowed for viewers too; Resume needs the operator role (see 9.5).
- **sindrava:** E-STOP is always visible, never needs confirmation, and reports success only after telemetry shows `emergency_stopped`. Stop never needs confirmation. Resume must be a deliberate action (see 9.5).

## 8. Validation status

| Behaviour | Implemented | Simulated | Software-tested | Hardware-validated |
|---|---|---|---|---|
| Software E-stop | yes | yes | yes | no |
| Interlock (RP2040) | yes | — | firmware tests | no |
| Interlock (STM32F411) | **no** (#21) | — | — | no |
| Stop | yes | yes | yes | no |
| Stop during planning wins | yes (#22) | yes | yes | — |
| Disable | yes | yes | yes | no |
| Fault reaction | yes | yes | yes | no |
| Panic safe state | yes (#68) | — | build only | no |
| Pause / Resume | **no** (#29) | — | — | — |

## 9. Decisions needed before #29

Each item has a recommendation; the owner decides.

1. **Should Stop become path-preserving?** Today Stop lets each axis brake on its own and may leave the path. Recommendation: once #29 exists, Stop = pause along the path + discard the rest, with per-axis `stop_decel` braking kept only as the MCU's own fallback (link loss, re-timing not possible in time). Trade-off: slightly longer stopping distance on some axes in exchange for staying on a known, collision-checked path.
2. **Pause during homing?** Recommendation: not allowed; Pause during homing acts as Stop (axis stays unhomed), because a homing search has no meaningful "continue".
3. **Resume after a long pause?** Recommendation: no timeout; Resume stays valid as long as the state revision is unchanged. Drives stay enabled and the communication watchdog keeps running while paused.
4. **Re-validate before Resume?** Recommendation: yes — limits and the collision check on the remaining path, so a model change (tool, obstacles) between pause and resume is caught.
5. **Roles and confirmation:** Recommendation: Pause allowed for viewers (like Stop); Resume needs the operator role and a deliberate UI action (button press, not keyboard auto-repeat), without a confirmation dialog.
6. **Stop latency target:** Is "deceleration starts within one motion tick of the frame reaching the MCU" the right requirement, or should Stop also have an end-to-end target from the UI?
