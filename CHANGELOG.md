# Changelog

All notable changes to MotionCore. Each capability is listed with its validation level:

- *implemented*: the code exists
- *simulated*: runs end to end on the virtual MCU
- *software-tested*: covered by automated tests in CI
- *hardware-validated*: measured on physical hardware

**Nothing in this release is hardware-validated.** The RP2040 and STM32F411 firmware compiles and is host-tested, but has not been measured on a bench yet (#25). MotionCore is not a certified safety system: always use an independent hardware E-stop that removes driver power.

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
- The collision check samples the path (#77).
- The STM32F411 has one axis (#46) and no interlock input (#21).

[0.3.0]: https://github.com/bzy-logics/motioncore/releases/tag/v0.3.0
