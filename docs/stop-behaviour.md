# Stop, pause, resume and emergency stop

- **Status:** Accepted by the project owner on 2026-11-19 ([#20](https://github.com/bzy-logics/motioncore/issues/20)). This document is the **authoritative contract** for the engine, the MCU firmware, the HAL, the ratavyr API and the sindrava UI. Where the implementation differs, the implementation is wrong; section 10 lists the known gaps and the issues that close them.
- **Applies to:** engine (`crates/core`, `crates/safety`), firmware (`crates/firmware-core`, `crates/firmware-stepper`, `boards/*`), HAL, ratavyr, sindrava, MCL programs.
- **Scope of claims:** nothing in this document makes MotionCore a safety-rated control system. Software stop functions are operational functions; the emergency stop is hardware.

## 1. Principles

1. **Stop, Pause, Resume and Emergency Stop are not interchangeable.**
   - **Stop:** controlled termination of motion.
   - **Pause:** temporary feed hold with a recoverable execution state.
   - **Resume:** validated continuation of the remaining trajectory.
   - **Emergency Stop:** independent safety action that takes precedence over every software motion command.

   An implementation must not substitute one for another (for example implement Pause as Stop plus a new move, or Stop as E-stop) except where this document says so explicitly (Pause during homing, section 6.2).
2. **The emergency stop is hardware.** The physical E-stop chain removes driver power independently of the MCU, the host, the gateway and the browser. Software permissions, software state and software faults must never prevent or delay a hardware emergency stop.
3. **Software E-stop has precedence** over every other command and state, and latches.
4. **Nothing resumes by itself.** Resetting a fault or an E-stop never restarts motion; a stopped or faulted move is never continued. Only an explicit Resume of an explicitly paused move continues motion, and only after the checks of section 7 pass.
5. **An unchanged software state does not prove that the machine is safe.** Resume and every other continuation verify the physical preconditions (position confidence, drives, interlocks, unexpected movement), not only the engine's bookkeeping.
6. **Position confidence is explicit** (section 5). Motion that depends on a known position is refused when the position is not trusted.
7. **Stop always wins over a move being planned** (#22): a Stop or E-stop that arrives while a move is being planned cancels it; nothing is sent.
8. **Software measurements never support a safety-rated stopping time.** Measured latencies and distances (section 8) are operational figures for this setup only.

## 2. Definitions

| Function | Purpose | Drives | Path | Execution state afterwards | Position confidence afterwards |
|---|---|---|---|---|---|
| **E-stop (hardware)** | Remove motion energy in an emergency | Power removed by the safety circuit | Not controlled | Discarded | Lost |
| **E-stop (software)** | Immediate halt from the UI, API or CLI | ENABLE released at once; step output halted | Not controlled | Discarded | Lost |
| **Stop** | Controlled termination of the current motion | Stay enabled | Coordinated stop **along the planned path** (section 4); reported fallback otherwise | Discarded | Unchanged |
| **Pause** | Feed hold that can be resumed | Stay enabled | Decelerates along the planned path | Kept (remaining trajectory) | Unchanged |
| **Resume** | Validated continuation of a paused motion | Stay enabled | Continues the remaining planned path | Running again | Must be Known (section 7) |
| **Disable** | Controlled Stop, then de-energise | Released after the stop | As Stop | Discarded | See section 5.2 |
| **Fault** | Reaction to a detected error | Released at once (MCU) | Not controlled | Discarded | Lost |
| **Reset fault** | Acknowledge a fault or E-stop | Stay disabled | — | — | Unchanged (still Lost) |

Mapping to IEC 60204-1 stop categories, as a guide only: hardware E-stop ≈ category 0; Stop and Pause ≈ category 2; Disable ≈ category 1. Releasing a driver's ENABLE input is not Safe Torque Off unless the driver certifies it.

## 3. Current behaviour (2026-11-19)

This section records what the code does today. Differences from the contract are listed in section 10.

### Software E-stop

- **Engine:** in any state, even without an MCU link: discards all planned and queued motion, marks every axis unhomed and disabled, latches `emergency_stopped`, then sends `EmergencyStop` to the MCU on the priority lane (ahead of queued segments).
- **MCU:** halts step generation and releases every ENABLE at once; latches `EmergencyStop`.
- **RP2040:** the GP27 interlock input (E-stop chain monitoring contact) trips the same latch when it opens, and blocks Enable and fault reset while open. **The STM32F411 has no interlock input yet** (#21).

### Stop

- **Engine:** discards planned motion that has not been sent (`clear_motion`), cancels a home-all run, and sends `Stop` to the MCU. The robot stays `running` until fresh MCU status shows every axis at rest with an empty queue, then returns to `ready`; commanded positions are resynced from the MCU step counters. If homing was in progress, the axis stays unhomed.
- **MCU:** clears every axis' segment queue and aborts homing; each moving axis decelerates from its current velocity at its own `stop_decel` (profile `stop_deceleration`, default 2 × `max_acceleration`). Drives stay enabled.
- **Consequence:** axes stop at different times, so **a coordinated (MoveJ, MoveJPath) or Cartesian (MoveL) move generally leaves its planned path while stopping.** The deviation is bounded by each axis' braking distance `v² / (2 · stop_decel)`. This is the fallback behaviour of section 4.3, used today in every case.

### Disable

Controlled Stop as above; once the MCU reports every axis idle (timeout 3 s, then fault), every ENABLE is released and the state becomes `disabled`. Axes stay homed.

### Faults

Any fault (MCU communication watchdog, queue underrun, late segment, endstop hit while moving towards it, soft limit, step-rate overrun, homing failure, plugin failure, link loss) latches `faulted`: the MCU releases every ENABLE immediately, the engine discards all motion and marks every axis unhomed. Fault reset is refused while the hazard persists (for example an endstop still pressed with no known safe direction, or the interlock open) and always lands in `disabled`.

### Firmware panic

Drives are disabled and pulse output stopped immediately, then the hardware watchdog resets the MCU (#68, pending hardware measurement). The engine sees the link loss as a fault.

### State × action

| State | Stop | Disable | E-stop | Reset fault |
|---|---|---|---|---|
| disconnected | refused (no MCU) | refused | latches `emergency_stopped` | refused |
| initializing | accepted, no effect | refused | latches | refused |
| disabled | accepted, no effect | no change | latches | refused |
| ready | accepted; cancels a move being planned | → disabled | latches | refused |
| homing | homing aborted; axis unhomed; → ready when idle | stop, then → disabled | latches | refused |
| running | controlled stop; → ready when idle | stop, then → disabled | latches | refused |
| faulted | refused by the MCU (nothing moving) | no change | latches `emergency_stopped` | → disabled if the hazard is gone |
| emergency_stopped | refused by the MCU | no change | stays latched | → disabled if the hazard is gone |

Motion commands (jog, move, home, MoveJ, MoveJPath, MoveL) are accepted only in `ready`; while a Stop is in progress the state is `running`, so they are refused until the robot is at rest. The `paused` state (section 6) adds a row with #29: Stop → discard, → ready; Disable → stop, then → disabled; E-stop latches; Pause again → no change; Resume → section 7.

## 4. Stop

### 4.1 Required behaviour

Stop terminates the current motion with a **coordinated controlled-stop trajectory** that:

1. follows the planned path: the stop trajectory is a prefix of the planned path with the path parameter decelerating monotonically to zero (it never reverses and never leaves the path, within step quantisation);
2. respects every axis' velocity limit and its stopping acceleration limit (`stop_decel`), and the configured jerk limits where jerk-limited profiles are supported (#80);
3. preserves coordinated motion as closely as physically possible — for a Cartesian move, the tool stays on the planned Cartesian path;
4. ends in a stopped state with drives enabled and no further motion: no creep, no re-approach, no correction move.

The remaining trajectory is discarded; Stop never leaves a resumable motion behind (that is Pause).

For a single-axis motion (jog, MoveTo of one axis) the coordinated stop and per-axis braking are the same thing.

### 4.2 Mechanism

The contract does not fix where the stop trajectory is computed. Two options satisfy it:

- **MCU time scaling:** the MCU slows a common time base for all axes of the motion to zero (a feed-override ramp). Because every axis follows the same time warp, the path geometry is preserved exactly; the ramp must be limited so that every axis stays within `stop_decel`.
- **Host re-timing:** the engine re-times the remaining trajectory (as for Pause) and replaces the queued segments from a cut point ahead of the MCU.

Whichever is used must meet the latency targets of section 8 and fall back as in 4.3 when it cannot.

### 4.3 Fallback and reporting

If a coordinated stop cannot be generated or delivered safely — for example the MCU link is lost, the re-timed trajectory is not ready before the cut point, or no feasible coordinated deceleration exists within `stop_decel` — the MCU brakes each axis independently at its `stop_decel` (the current behaviour, section 3).

When the fallback is used, the engine **must report that path preservation was not achieved**: in the Stop result, in the motion's execution event (#28) and in telemetry (`path_preserved: false`), with the reason. The UI shows it. A Stop whose path preservation is unknown counts as not preserved.

### 4.4 Stop during homing

Homing is aborted, the axis being homed stays unhomed, and the home-all run is cancelled.

## 5. Position confidence

### 5.1 Levels

Every axis has a position confidence, reported in telemetry and by the API:

| Level | Meaning | Allowed |
|---|---|---|
| **Known** | The position is sufficiently trustworthy for planned motion. | All motion, Resume |
| **Uncertain** | The position may have changed unexpectedly. | Homing, jog (operator, reduced speed); **no** Resume, no planned or Cartesian motion that relies on the position |
| **Lost** | The position cannot be trusted. | Homing only (and jog where the profile permits jogging an unhomed axis) |

The robot's confidence is the lowest of its axes'. Confidence is raised only by homing, or by a recovery procedure the administrator has configured (section 9.3); it is never raised by Reset fault, by re-enabling, or by time passing.

**Open-loop stepper limitation:** without encoders, MotionCore cannot detect missed steps. "Known" then means "the step counter is trusted and no event below has occurred"; it does not prove that the axis followed its steps. Encoder feedback (#42) and following-error monitoring (#43) can lower confidence when a deviation is measured.

### 5.2 Events that lower confidence

| Event | Resulting confidence |
|---|---|
| Hardware E-stop, software E-stop | Lost (all axes) |
| Power loss, MCU reset, firmware panic, MCU link loss | Lost (all axes) |
| Fault that releases the drives | Lost (all axes) |
| Driver fault or driver alarm on an axis | Lost (that axis; all axes if the drives are released) |
| Uncontrolled mechanical movement detected (encoder, following error #43) | Lost (that axis) |
| Missed steps or stall detected | Lost (that axis) |
| Disable on an axis **not** marked self-locking in the profile | Uncertain (that axis): holding torque is gone and gravity or external force may move it |
| Disable on an axis marked self-locking (lead screw, worm gear, brake) | Unchanged |
| Following error above a warning threshold, below the fault threshold | Uncertain (that axis) |
| Stop with the per-axis fallback (4.3) | Unchanged — positions are still counted; only path preservation is lost |
| Stop, Pause (coordinated) | Unchanged |

Today the engine has two levels only: homed (Known) and unhomed (Lost). Uncertain, the per-axis self-locking flag and the Disable rule are new (section 10).

## 6. Pause

### 6.1 Required behaviour

Pause is a feed hold: the remaining trajectory is re-timed so that the path parameter decelerates to zero while every axis stays on the planned path (the same coordinated deceleration as Stop, section 4.1, using the normal acceleration limits, not `stop_decel`), then the motion holds with drives enabled.

- Allowed in `running` for MoveJ, MoveJPath, MoveL, jog and MoveTo.
- State: `paused`, reported in telemetry with the paused motion's id (#28), the held point and the position confidence.
- Stop while paused discards the paused motion (→ `ready`). E-stop, fault and Disable discard it permanently.
- If the deceleration cannot be kept on the path, Pause behaves as Stop with the fallback, reports `path_preserved: false`, and leaves no resumable motion.

### 6.2 Pause during homing

Pause during homing **acts as Stop** (section 4.4): homing is a calibration operation, not ordinary trajectory execution, and resuming halfway through it could invalidate the reference position. The axis stays unhomed and homing must be restarted from the beginning (or follow an explicitly defined recovery procedure). The API reports that Pause was executed as Stop.

### 6.3 MCL programs

Pause pauses the program at the current motion instruction; Resume continues it after the checks of section 7. Stop ends the program. A fault or E-stop ends the program and reports the instruction and source line.

## 7. Resume

Resume continues the paused motion from the held point to its original target, along the original path, with the original speed limits. **There is no time-based timeout**; instead, Resume verifies at the moment it is requested that the robot is still physically safe to continue. All of the following must hold:

1. **Position validity:** every axis of the motion is **Known** (section 5). A robot that has lost position confidence never resumes; it needs homing or the configured recovery procedure, after which the paused motion is gone.
2. **Motor and drive readiness:** drives enabled, MCU link up in the same MCU session as the pause (no reset or reconnection), no driver alarm.
3. **No active faults or interlocks:** state `paused`, interlock closed, no latched or pending fault.
4. **No unexpected movement during the pause:** the MCU step counters equal the held point; with encoders, the measured position is within the configured tolerance of the held point.
5. **Remaining trajectory validity:** the remaining trajectory is revalidated against the current
   - joint limits and soft limits,
   - kinematic configuration (the same IK configuration and solution branch along the remaining path),
   - collision geometry (workcell model),
   - tool and workpiece configuration (TCP, tool geometry, payload),
   - safety constraints (speed and acceleration limits, scaling, zones).
6. **No relevant configuration or environment change:** the robot profile, kinematic model, collision model, tool and frames have the same revision as when the motion was planned; any change fails this check even if the remaining path would still pass 5.

The engine's state revision (#22) must also be unchanged apart from the Pause itself; a revision change is a sufficient reason to refuse, but an unchanged revision is not sufficient to accept.

If any check fails, Resume is **refused** with `invalid_state` and the failed check(s), the paused motion is discarded, and the robot returns to `ready` (or stays in its fault state). Continuing requires a new move, preceded by homing or the configured recovery procedure where position confidence requires it.

## 8. Latency and stopping-distance requirements

### 8.1 Metrics

Every stop function is measured as four separate quantities:

| # | Metric | From | To |
|---|---|---|---|
| L1 | Command receipt latency | Request accepted at the API boundary (ratavyr, or the engine API for local clients) | Command frame received by the MCU |
| L2 | Reaction time | Command frame received by the MCU (or input edge for the interlock) | Deceleration begins (first step interval longer than planned) — for E-stop: ENABLE released and last STEP edge |
| L3 | Stopping time | Deceleration begins | Every axis at zero velocity |
| L4 | Stopping distance | Command frame received by the MCU | Rest, per axis and, for Cartesian motion, of the TCP along and off the path |

Each metric is measured **under the worst-case supported operating conditions**: maximum configured velocity, maximum configured load, maximum configured acceleration, and the worst supported host load. Results are reported per board and per robot profile with the measurement setup.

### 8.2 Targets

| Function | L2 target | L3 / L4 |
|---|---|---|
| Software E-stop | ENABLE released and no further STEP edge within **1 ms** | Not controlled (drives released); L4 measured and reported |
| Interlock opening | As software E-stop, from the input edge | As software E-stop |
| Firmware panic | ENABLE released within 10 µs (RP2040) / 50 µs (STM32) | As software E-stop |
| Communication loss | Drives released within the profile's `watchdog_ms` | As software E-stop |
| Stop (coordinated) | Deceleration begins within the bound set by the chosen mechanism (4.2), at most **20 ms** after the frame reaches the MCU; otherwise the fallback is used | L3 and L4 within the planned stop trajectory's time and distance + step quantisation |
| Stop (fallback) | Deceleration begins within **one motion tick** (200 µs RP2040, 50 µs STM32) | Each axis within `v² / (2 · stop_decel)` |
| Pause | As coordinated Stop | Within the re-timed deceleration's time and distance |

L1 crosses non-real-time host software (browser, gateway, engine, USB). It is measured and reported but **not bounded**, and must not be relied on for safety.

These targets are operational. **No safety-rated stopping time or distance may be claimed from software measurements**; a safety-rated stop needs the hardware chain, certified components and a separate assessment (#78).

## 9. Interfaces and permissions

### 9.1 Engine API

`Stop`, `EmergencyStop`, `Disable` and `ResetFault` exist. `Pause` and `Resume` are added with #29; Stop and Pause results gain `path_preserved` and a reason; Resume refusals list the failed checks; telemetry gains per-axis position confidence. All additive (version per the versioning rules).

### 9.2 Roles (ratavyr)

| Action | Who |
|---|---|
| Software E-stop | Any authorized user (viewer and above) |
| Stop | Any authorized user (viewer and above) |
| Pause | Operator and above; **viewer only if the machine's operational policy permits it** (policy set by an administrator; default: not permitted) |
| Resume | Operator and above |
| Reset fault, Enable, Disable, motion | Operator and above |
| Configure recovery and reset policies (viewer Pause, recovery procedures that restore position confidence, which roles may reset faults) | Administrator |
| Hardware E-stop | Independent of every application permission |

**Software permissions must never prevent a hardware emergency stop**, and must never be required for a software E-stop or Stop beyond being an authorized user of the robot.

### 9.3 Recovery procedures

A recovery procedure restores position confidence without full homing (for example: move to a reference mark and confirm). Recovery procedures are configured by an administrator per robot, are recorded in the event log with the user who performed them, and are never run automatically. Until one is configured, homing is the only recovery.

### 9.4 sindrava

- E-STOP is always visible, never needs confirmation, and reports success only after telemetry shows `emergency_stopped`.
- Stop never needs confirmation; when Stop reports `path_preserved: false`, the UI says so.
- Pause and Resume are separate controls; Resume is a deliberate action (button press, not keyboard auto-repeat), shown only to roles that may use it, and shows the failed checks when refused.
- Per-axis position confidence is visible; Uncertain and Lost axes are highlighted and motion controls that need Known are disabled.

## 10. Validation status and gaps

| Behaviour | Implemented | Simulated | Software-tested | Hardware-validated |
|---|---|---|---|---|
| Software E-stop | yes | yes | yes | no |
| Interlock (RP2040) | yes | — | firmware tests | no |
| Interlock (STM32F411) | **no** (#21) | — | — | no |
| Stop, per-axis fallback | yes | yes | yes | no |
| Stop, coordinated along the path (4.1) | **no** (#102) | — | — | no |
| `path_preserved` reporting (4.3) | **no** (#102) | — | — | — |
| Stop during planning wins | yes (#22) | yes | yes | — |
| Disable | yes | yes | yes | no |
| Fault reaction | yes | yes | yes | no |
| Panic safe state | yes (#68) | — | build only | no |
| Position confidence Known / Lost (homed / unhomed) | yes | yes | yes | — |
| Position confidence Uncertain, self-locking axes (5) | **no** (#103) | — | — | — |
| Pause / Resume with the checks of section 7 | **no** (#29) | — | — | — |
| Roles and recovery policies (9.2, 9.3) | partly (viewer Stop/E-stop) — rest [ratavyr#17](https://github.com/bzy-logics/ratavyr/issues/17) | — | — | — |
| UI for confidence, Pause/Resume, path preservation (9.4) | **no** ([sindrava#14](https://github.com/bzy-logics/sindrava/issues/14)) | — | — | — |
| Latency metrics L1–L4 (8) | — | — | — | **not measured** (#104) |

## 11. Decision record

Decided by the project owner on 2026-11-19 (#20):

1. **Stop preserves the planned path — yes.** Coordinated controlled stop (section 4) with a defined, reported fallback.
2. **Pause during homing — no; it acts as Stop.** Homing restarts or follows a defined recovery procedure (6.2).
3. **Resume timeout — conditional.** No time-based timeout; Resume verifies position validity, drive readiness, faults and interlocks, unexpected movement, remaining-trajectory validity and configuration changes (section 7). A robot that has lost position confidence never resumes automatically.
4. **Revalidate the remaining trajectory — yes.** Joint limits, kinematic configuration, collision geometry, tool and workpiece, safety constraints; failure refuses Resume and requires recovery (7.5).
5. **Roles — refined** as in 9.2: viewer Pause only by policy; operator Pause and Resume; administrator configures recovery and reset policies; any authorized user may Stop; hardware E-stop independent of permissions.
6. **Stop latency — yes, with measurable criteria:** four metrics L1–L4 under worst-case conditions; no safety-rated claims from software measurements (section 8).
7. **Additional requirement — position confidence** (Known / Uncertain / Lost) with Resume prohibited unless the required confidence is restored (section 5). The event mapping in 5.2 was confirmed by the owner on 2026-11-19.

Also confirmed by the owner on 2026-11-19: the 20 ms bound for the coordinated Stop's reaction (8.2), viewer Pause off by default (9.2), and a refused Resume discarding the paused motion (section 7).
