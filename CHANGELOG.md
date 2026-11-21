# Changelog

All notable changes to MotionCore. Each capability is listed with its validation level:

- *implemented*: the code exists
- *simulated*: runs end to end on the virtual MCU
- *software-tested*: covered by automated tests in CI
- *hardware-validated*: measured on physical hardware

**Nothing in this release is hardware-validated.** The RP2040 and STM32F411 firmware compiles and is host-tested, but has not been measured on a bench yet (#25). MotionCore is not a certified safety system: always use an independent hardware E-stop that removes driver power.

## [Unreleased]

### Changed — internal, software-tested

- One trajectory type (ADR 0003 stage 2, #91). `Trajectory`: per-joint piecewise polynomials of degree ≤ 3 on shared breakpoints, with motion kind, limits and speed scale, evaluated for position, velocity, acceleration and jerk at any time. Lossless conversions from every planner's output: `from_profiles` (trapezoids, synchronised moves, blended paths) and `from_sampled` (time-scaled paths, as cubic Hermite through the knots). Property tests, 1000 cases per planner, check the conversions. The engine does not use it yet; no external change.
- One kinematics interface (ADR 0003 stage 1, #90). `KinematicModel` (geometry, limits, frames, tool pose, Jacobian, `ik_near`, optional `ik_all`) with one forward-kinematics implementation, `joint_frames`; the serial `Chain` and the new `JointTree` implement it. The collision model now takes its frames from the robot's kinematic model instead of computing its own, so the solver and the collision check cannot disagree about where a link is. The unused `Kinematics` trait and `JointIdentity` are removed; the builtin `joint_identity` plugin keeps its manifest and capabilities. No change to the engine API, the protocol or robot profiles.

### Documentation

- ADR 0003, motion architecture (`docs/adr/0003-motion-architecture.md`): one `KinematicModel`, one `Trajectory` type, planners behind one interface, validation as an explicit stage with a report, execution separate from planning, dependency rules enforced in CI; eight refactor stages #90–#97 (#79).
- Safety concept (`docs/safety-concept.md`): hazards, and every protection with its layer (offline validation, runtime monitoring, hardware only), coverage, limits and validation status. Linked from the safety guide, the stop specification and the HAL ADR (#78).

### Added — software-tested; not hardware-validated

- Bench test builds from committed code: `motioncore firmware build --bench bench-isr-probe` (GP28 / PB1 high while the motion tick interrupt runs) and `--bench bench-panic`. Flash and verify take the same `--bench` option (#114).
  - The feature is part of the lockfile and build hash. Bench builds go to their own artifact directory.
  - The firmware reports the feature as a new capability bit (`bench.isr_probe`, `bench.panic`). `firmware verify` prints the build type and the engine warns when it connects to a bench build.
  - Production build hashes are unchanged. The protocol version is unchanged: older hosts ignore the new bits.
- `motioncore firmware rate-test`: the RP2040 test 8 step-rate sweep, talking to the MCU directly because the engine caps moves at the advertised rate. It refuses to run without `--no-motor-power` or while an engine is reachable (#114).
- CI lints every bench feature on both boards and checks that the two features sharing a pin can't be built together (#114).

### Changed — software-tested

- The collision check is certified between the poses it evaluates (#77). It bounds how fast each joint can move any point of a link and steps only as far as the remaining clearance allows, so the 2 mm margin holds along the whole planned path of every move type, not only at samples. Steps are as long as before far from contact and shorter near it.
  - MoveJPath checks the lines between its 5 ms samples widened by how far the blended curve can bow away from them.
  - A path that can't be shown to keep the margin to within 0.01 mm is refused, with a message saying the links "could come within" the reported distance.
  - Accepted moves log the certified clearance.
  - A robot already closer than the margin may still back out, but no such gap may shrink by more than 0.1 mm or end smaller than it started. Before, any move that ended no worse than the start's overall minimum was allowed.
  - Property test: accepted random paths, resampled every 0.01°, never come closer than the margin or the reported clearance.
  - The guarantee is for the capsule model and the planned joint path, not for how closely the motors track it.

### Fixed — software-tested

- The engine could not connect to an MCU that already had a latched fault: the safety interlock open at boot, or the watchdog expired while the host was away (an engine restart). The MCU refuses axis configuration while faulted, so every reconnect failed until the board was power-cycled. The engine now reads the MCU's status at connect and, when it is faulted, connects into `faulted` with the MCU's fault; Reset fault configures the axes once the hazard is gone. Nothing is reset automatically. Found on an RP2040 board with the interlock input open.
- A failed connection attempt left the serial port open, so every retry failed with "Access is denied" on Windows (Linux allows reopening a tty, which hid it). The engine now closes the link (`McuLink::close`) whenever it drops it.
- The RP2040 reference profiles (`six-axis-rp2040-pico.toml`, `six-axis-rp2040-pico-timing.toml`) named a robot plugin that does not exist (`org.motioncore.robot.six_axis`), so the engine refused to start with them; they now use `org.motioncore.robot.single_axis` like the other profiles. This changes the build hash of `--board rp2040-pico` builds (the profile is part of the lockfile); the firmware image is unchanged. A test now checks the plugins of every shipped profile.
- MoveL was refused from a pose with a joint resting at its soft limit: the step-quantised position sits a fraction of a step outside (e.g. 180° at 213.333333 steps/° reads 180.00000028°), and the start point was checked against the limits too (#23).
- MoveL could send a segment above the MCU's step-rate limit when an acceleration ramp ended between two 10 ms knots; the MCU refuses such a segment mid-move. Knots now also sit on the ramp boundaries (#23).
- MoveJPath lagged its plan by up to ~1.7 steps where a joint reverses inside a blended corner: the MCU cannot run one segment backwards then forwards. Segments are now split at velocity reversals (#24).
- `tools/step-skew` failed correct timing on slow axes: its jitter limit was a fixed 250 µs, but where a channel steps slowly one step is longer than that. Each edge may now deviate one window plus one step at its local step rate; jitter is reported in µs and steps. RP2040 test 15 uses the new criterion (#110).
- `examples/sim-vision-pick` used a guessed command name and never ran MoveJ; it now uses `move_j` and, for descend and lift, `move_l` (#107).

### Tests

- Property-based planner tests: single-axis moves, MoveJ, MoveJPath and MoveL time-scaling, 1000 random cases each, checked against the kinematic limits and the MCU's own segment rules (#23).
- Simulated execution timing: the firmware runtime on a simulated clock runs MoveJ, MoveJPath and MoveL on the six-axis arm within ±1 step and one tick of the plan; Stop decelerates at the configured rate; no step after an E-stop or a link-loss fault (#24). `tools/step-skew` is checked against this ground truth.

## [0.3.0] — 2026-11-19

The first MotionCore release, and the baseline that v0.4 (verified motion on hardware) is measured against.

### Compatibility

| Component | Version |
|---|---|
| Engine API (`motioncore.engine.v1`) | `0.3.0` (tag `engine-api/v0.3.0`); clients accept `0.3.x` engines |
| MCU protocol | `PROTOCOL_VERSION` 1 (engine and firmware must match) |
| Plugin API | `"0"` (unstable; WASM plugins are not executed) |
| ratavyr | v0.3.0 (pins `engine-api/v0.3.0`) |
| sindrava | v0.3.0 (tested against ratavyr v0.3.0) |

### Motion — simulated, software-tested

- Jog, single-axis moves and homing with trapezoidal profiles.
- Home-all plans: groups of axes homed together, in a configured order.
- `MoveJ`: coordinated joint moves; all axes start and finish together.
- `MoveJPath`: blended joint paths through up to 256 waypoints without stopping at intermediate ones.
- `MoveL`: straight Cartesian line of the tool point, with orientation held or given, for serial-chain geometries; time-scaled within every joint's velocity and acceleration limits.
- Kinematic geometry in robot profiles; tool pose in telemetry.
- Collision checking (self-collision and link thickness) before a move is sent. Poses are checked at samples along the path; clearance between samples is not guaranteed yet (#77).
- Moves are planned outside the engine lock: telemetry and Stop stay responsive while a long move is planned, and a Stop or E-stop that arrives during planning cancels the move before anything is sent (#22).

### Engine and safety — simulated, software-tested

- Engine as the single state authority, with gRPC `EngineService` and `FirmwareService`.
- Supervisor state machine: disconnected, initializing, disabled, ready, homing, running, faulted, emergency_stopped. Faults and E-stop latch; reset never resumes motion and requires re-homing.
- Software E-stop with priority delivery to the MCU.
- Stop: today each axis brakes on its own at `stop_decel`, so a coordinated or Cartesian move may leave its path while stopping. A path-preserving stop is specified (#102).
- Stop, pause, resume and E-stop specification, accepted as the contract for every component: [`docs/stop-behaviour.md`](docs/stop-behaviour.md) (#20). Pause and resume are not implemented yet (#29).
- Debug fault-injection RPCs for tests (`motioncore start --enable-fault-injection`; never with real hardware).

### Firmware — host-tested, cross-compiled; not hardware-validated

- One firmware core for every board and the simulator: protocol (COBS framing, CRC), segment execution, homing, communication watchdog, soft limits, endstops, step-rate ceiling, fault latching.
- RP2040 (Raspberry Pi Pico): up to 6 axes, PIO-timed STEP/DIR with guaranteed pulse widths and direction setup/hold; E-stop chain monitoring input on GP27.
- STM32F411 (Black Pill): 1 axis, timer-interrupt step output. No E-stop chain input yet (#21).
- Safe state on firmware panic: drives disabled and pulse output stopped before halting (#68; implemented and built in CI, not yet measured on hardware).
- Firmware builder: reproducible builds pinned by a lockfile, checksummed artifacts, flash (UF2 or probe-rs; refused while drives are enabled) and verify over the wire.

### Plugins — implemented, software-tested

- Plugin SDK and runtime: manifests, permissions, dependency resolution and lifecycle; builtin reference plugins. WASM plugins are not executed yet (#55).

### Tools and documentation

- `motioncore` CLI for the engine, simulator and firmware tools.
- `tools/step-skew` for measuring channel-to-channel timing on the RP2040 (measurement pending, #13).
- `examples/sim-vision-pick`: a simulated camera and object detector drive an arm through the public REST API.
- CI on every pull request: formatting, clippy, tests, both firmware targets, the panic test build, the step-skew self-test and the example (#18).
- Architecture decisions: HAL (ADR 0002). Documentation site: [motioncore.bzy-logics.com](https://motioncore.bzy-logics.com).

### Known limitations

- Nothing is hardware-validated (#25, #26, #104).
- Stop does not preserve the path (#102); no pause and resume (#29).
- Position confidence has two levels only, homed and unhomed (#103).
- The collision check covers the robot's own links and the floor, not the rest of the cell; it is a pre-move check, not a runtime monitor.
- The STM32F411 has one axis (#46) and no interlock input (#21).

[0.3.0]: https://github.com/bzy-logics/motioncore/releases/tag/v0.3.0
