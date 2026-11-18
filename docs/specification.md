# MotionCore — Project Architecture and Development Specification

**Status:** Proposed design, not an implemented product  
**Version:** Draft v0.2  
**Core vision:** A plugin-first, hardware-independent robotics motion-control platform delivered as three independent repositories: `motioncore` (Rust engine, firmware, plugins and builder), `ratavyr` (Go API), and `sindrava` (Vite + React interface).

> Implementation status against this specification is tracked in
> [architecture.md](architecture.md) and the top-level README.

## 1. Executive Summary

MotionCore aims to let users configure, build firmware for, connect, monitor, and control supported robotic systems without rewriting the entire motion-control stack for each robot. Its first physical reference is the Annin Robotics AR4-MK5, but the architecture must not depend on that model. MotionCore runs independently of ROS; ROS 2 and AI simulation are separate future portfolio projects.

**Primary differentiators:**
1. Plugin-first host runtime and extensible robot/device capabilities.
2. Host/MCU separation with custom, buildable Rust firmware.
3. A device graph and visual Robot Composer that can validate compatible components.
4. A unified virtual-MCU and real-hardware control interface.
5. Browser-based firmware build, flashing, telemetry, diagnostics, and robot visualization.

## 2. Goals and Non-goals

### Goals
- Linux-hosted robotics runtime.
- Reusable robot, kinematics, planning, actuator, sensor, tool, and integration plugins.
- Build and flash supported MCU firmware from versioned, reproducible configurations.
- Time-synchronized, buffered commands between host and MCU.
- Multi-axis joint-space motion with strict validation.
- Configurable robot geometry, gearing, limits, devices, and homing procedures.
- Go HTTP/WebSocket API and React/Vite dashboard.
- Simulation and hardware sharing the same high-level command interface.
- Safety-conscious fault handling, audit logs, and diagnostics.

### Explicit non-goals for v0.1
- Full ROS 2 dependency or ROS-based core control.
- General-purpose arbitrary-board firmware synthesis.
- Public marketplace and execution of untrusted native plugins.
- Production-certified functional safety.
- Fully autonomous operation, advanced obstacle avoidance, or AI training.
- Immediate support for STM32, ESP32, and Teensy simultaneously.

## 3. Technology Stack

| Layer | Proposed technology | Responsibility |
|---|---|---|
| Web frontend | Vite, React, TypeScript, Three.js | Configuration, dashboards, robot visualization |
| Web backend | Go, Gin or net/http, WebSocket | API, authentication, build-job management, streaming |
| Host motion engine | Rust, Tokio, nalgebra | State, kinematics integration, planning, scheduling |
| Host plugin runtime | Rust, Wasmtime/component model | Discovery, lifecycle, capabilities, isolated extensions |
| Go ↔ Rust | Versioned gRPC over local Unix socket | Control plane and telemetry |
| MCU protocol | Rust, postcard or other compact codec, COBS + CRC | Framed transport, commands, telemetry, version negotiation |
| Firmware | Rust `no_std`, embedded-hal, target-specific HAL, optional Embassy | Hardware timers, outputs, feedback, watchdog |
| Firmware builder | Rust CLI and controlled build workers | Configure, compile, flash, verify |
| Initial storage | Local configuration files and SQLite as needed | Profiles, jobs, plugin metadata; avoid premature database complexity |

**Important:** Real-time step generation belongs on the MCU. Go is a management gateway, not the authoritative motion executor.

## 4. Logical Architecture

```text
Browser: Vite + React + Three.js
            |
       REST / WebSocket
            |
       Go API Service
       |           |
       |     Firmware build job orchestration
       |
  versioned gRPC (local Unix socket)
       |
  Rust MotionCore Host
  +-- Plugin manager / capability registry
  +-- Device graph / configuration resolver
  +-- Robot state machine / command arbitration
  +-- Kinematics and trajectory processing
  +-- Motion validation / safety supervisor
  +-- MCU manager / synchronized command queues
  +-- Firmware builder CLI / isolated build worker
       |
  Versioned binary MCU protocol
       |
  Rust MCU firmware (STM32 first)
  +-- Clock, command buffer, watchdog
  +-- Timer-driven step/dir, PWM, GPIO
  +-- Encoder/endstop sampling and telemetry
       |
  Motors, drivers, sensors, end-effectors
```

The Rust engine is authoritative for motion state and hardware connections. The Go API never sends raw unvalidated step pulses directly to an MCU.

## 5. Plugin Architecture

### 5.1 Three extension layers

| Extension | Packaging | Execution | Examples |
|---|---|---|---|
| Host plugin | Versioned WASM component + manifest | Rust/Wasmtime, capability-restricted | Kinematics, robot profiles, automation, integrations |
| Firmware module | Rust crate + target feature | Statically compiled into MCU firmware | Stepper, encoder, GPIO, sensor drivers |
| Web extension | Versioned UI extension contract | Controlled React integration | Custom panels, settings, telemetry widgets |

For v0.1, web extensions can be declarative panels rather than arbitrary third-party JavaScript. Native Rust plugins, if later needed, should use isolated processes and versioned IPC instead of unstable shared-library Rust ABIs.

### 5.2 Host plugin categories
- Robot definitions and profiles.
- Kinematics (forward/inverse).
- Motion/path generation.
- Actuator and controller adapters.
- Sensors and feedback.
- Tools/end-effectors.
- Automation and macros.
- Integration adapters (future ROS 2, MQTT, etc.).

### 5.3 Manifest concept

```toml
id = "org.motioncore.kinematics.articulated6"
version = "0.1.0"
api_version = "0"
entrypoint = "plugin.wasm"
provides = ["kinematics.fk", "kinematics.ik"]
requires = ["robot.joint_model"]
permissions = ["robot.read_geometry"]
```

This is an illustrative format, not a finalized schema. Plugins need unique identifiers, semantic versions, dependency constraints, declared permissions, checksums/signatures where appropriate, and explicit compatibility rules.

### 5.4 Plugin lifecycle
`discover → validate → resolve dependencies → initialize → activate → monitor → deactivate`.

A plugin failure must not silently change the authoritative robot state. Plugins request actions through the core; the core validates motion, arbitrates ownership, and enforces limits. Firmware independently handles supported faults.

## 6. Device Graph and Robot Composer

Represent a robot as typed components and connections, not a hardcoded six-motor structure.

```text
Robot
 +-- Kinematic chain
 |    +-- Joint J1 -> actuator -> driver -> MCU channel
 |    |                         +-- encoder / homing switch
 |    +-- Joint J2 -> ...
 |    +-- Joint J6 -> ...
 +-- Tool / gripper
 +-- Controllers / MCUs
 +-- Sensors
 +-- Safety inputs and drive enables
```

Each node declares required/provided capabilities, resource consumption, pin assignments, units, and constraints. The resolver should detect unavailable timers, duplicate pin allocation, incompatible voltage assumptions (where board metadata supports them), missing firmware features, and mismatched encoder/driver capabilities.

**Robot Composer workflow:** create robot → select geometry/joints → add devices/controllers → resolve plugins → validate wiring/resources → generate firmware configuration → build/flash → calibrate → simulate → operate.

Automated compatibility checks cannot replace electrical verification or a physical safety assessment.

## 7. Motion Engine

### 7.1 Pipeline
1. Receive typed motion request (joint move, jog, home, stop).
2. Check robot operational state and command ownership.
3. Resolve robot geometry and kinematics as needed.
4. Produce candidate path/waypoints.
5. Apply trusted time parameterization and dynamic constraints.
6. Validate joint limits, finite values, velocities, acceleration, timing, and supported safety constraints.
7. Convert to synchronized actuator commands.
8. Queue timestamped commands to MCUs.
9. Observe feedback and fault conditions.

### 7.2 Standard trajectory representation

```rust
pub struct JointWaypoint {
    pub time_ns: u64,
    pub positions_rad: Vec<f64>,
    pub velocities_rad_s: Vec<f64>,
}
```

Illustrative host-side representation only. The production contract needs explicit units, bounded sizes, joint identities, monotonic timestamps, numeric validation, and MCU-specific fixed-point/step conversions. Prismatic joints require linear units, not radians.

### 7.3 Safety and arbitration
- One authoritative command owner per physical actuator or coordinated group.
- Plugin-generated trajectories are proposals, never trusted hardware commands.
- Hardware emergency-stop/drive-disable must function independently of Linux, Go, Rust host, and plugins.
- Explicit states: disconnected, initializing, disabled, ready, homing, running, faulted, emergency-stopped.
- Fault reset must not automatically resume interrupted motion.
- Collision safety requires geometry and environmental models; joint limits alone do not guarantee collision-free movement.

## 8. MCU Firmware Design

### 8.1 Responsibilities
- Deterministic hardware-timer-driven step generation and/or supported actuator outputs.
- Timestamped bounded command queue.
- Clock synchronization support.
- Endstop and encoder sampling.
- Motor enable/disable and configured fault responses.
- Watchdog, queue underrun detection, communication-loss handling.
- Capability and firmware-version reporting.

### 8.2 Initial target strategy
Start with **one specific STM32F4 board**, selected after confirming Rust HAL support, timer availability, flashing workflow, I/O voltage, and required channels. ESP32-S3 and Teensy 4.1 are later adapters; do not assume identical Rust support or flashing behavior.

### 8.3 Protocol concepts
- Framing: COBS (serial) plus CRC for corrupted-frame detection.
- Message version and feature negotiation.
- Device identity, firmware build hash, board ID.
- Sequence numbers and acknowledgements for appropriate commands.
- Time-sync exchange, bounded buffers, queue status.
- Heartbeats, faults, endstop/encoder telemetry.
- Idempotency/retry rules for configuration versus non-idempotent motion operations.

Example capability report:

```json
{
  "protocol_version": 1,
  "target": "stm32f4",
  "firmware_version": "0.1.0",
  "capabilities": ["stepper.step_dir", "encoder.quadrature", "gpio.input"],
  "limits": {"step_channels": 6, "encoder_channels": 4}
}
```

These values are illustrative, not guarantees for any board.

## 9. Firmware Builder and Flashing

### Proposed CLI

```bash
motioncore firmware targets
motioncore firmware configure
motioncore firmware build --target stm32f4
motioncore firmware flash --device /dev/ttyACM0
motioncore mcu list
```

Commands are planned API concepts, not currently implemented.

### Build pipeline
1. Choose a supported board and transport.
2. Resolve firmware modules from device graph requirements.
3. Validate pins, timers, peripheral conflicts, and resource budgets.
4. Generate a pinned build configuration and lockfile.
5. Compile using a controlled Rust toolchain.
6. Produce image, manifest, checksum, build logs.
7. Require operator confirmation and compatible bootloader/flash method.
8. Flash and verify reported firmware hash and capabilities.
9. Preserve recovery instructions and known-good artifacts.

Never execute arbitrary plugin-supplied build scripts with unrestricted host privileges. Avoid flashing while drives are enabled or motion is active.

## 10. Web Product

**React + Vite + TypeScript + Three.js**, served in production as static assets by the Go backend if desired.

Core screens:
- Overview: connected robot, state, alarms, active job.
- Robot Composer: joints, links, motors, MCUs, sensors, tools.
- Plugin Manager: discover, install, enable, configure, inspect permissions.
- Firmware Builder: board, modules, pins, build logs, flash and verify.
- Motion Control: jog, home, limits, command queue, safe stop.
- 3D Viewer: planned pose vs measured pose, frames, joint positions.
- Diagnostics: logs, protocol status, clock drift, buffer fill, faults.
- Settings: users, profiles, backups, firmware and plugin versions.

Use REST for configuration and WebSocket for live state. Treat Rust engine state as authoritative; Go should proxy and cache carefully, not create a competing state machine.

## 11. Repository Layout — Three Independent Repositories

MotionCore uses **three separate Git repositories**, with independent versioning, CI/CD, releases and ownership boundaries. Firmware is **not** a fourth repository: it is part of the Rust MotionCore project because the host, firmware and simulator share protocol types, capability definitions and compatibility requirements.

### 11.1 `motioncore` — Rust robotics runtime and firmware

```text
motioncore/
├── Cargo.toml                 # Rust workspace
├── Cargo.lock
├── rust-toolchain.toml
├── crates/
│   ├── core/                  # Runtime and authoritative robot state
│   ├── motion/                # Trajectory planning and scheduling
│   ├── kinematics/            # FK / IK abstractions and algorithms
│   ├── safety/                # Trusted motion validation
│   ├── device-graph/          # Capability and resource graph
│   ├── plugin-runtime/        # Plugin discovery, isolation, lifecycle
│   ├── plugin-sdk/            # Versioned extension contracts
│   ├── protocol/              # Shared host ↔ MCU wire types
│   ├── engine-api/            # Versioned Go ↔ Rust API contracts
│   ├── mcu-manager/           # Connections, clock sync, queues
│   ├── simulator/             # Virtual MCU
│   ├── firmware-core/         # Embedded runtime, no_std
│   ├── firmware-hal/          # Embedded hardware interfaces
│   ├── firmware-stepper/      # Build-time firmware module
│   ├── firmware-encoder/      # Build-time firmware module
│   ├── firmware-gpio/         # Build-time firmware module
│   └── firmware-builder/      # Configure, compile, flash, verify
├── boards/
│   └── stm32f4/              # Initial board-specific firmware target
├── plugins/                   # Reference host plugins
├── robot-profiles/            # Sample robot configurations
├── configs/
├── examples/
├── tests/
└── docs/
```

**Build boundary:** A single source repository and Rust workspace does **not** mean host and embedded crates must compile together. Board binaries use their own target triples, target-specific dependencies, linker settings and build commands. The firmware builder selects compatible modules at compile time. Add ESP32 and Teensy board targets only after validating their toolchain and HAL support.

### 11.2 `ratavyr` — Go management/API service

```text
ratavyr/
├── cmd/
│   └── server/
├── internal/
│   ├── api/                   # REST routes and validation
│   ├── auth/                  # Authentication and authorization
│   ├── engine/                # Versioned Rust engine client
│   ├── websocket/             # Telemetry streaming
│   ├── robots/                # Robot configuration workflows
│   ├── plugins/               # Plugin management endpoints
│   ├── firmware/              # Build/flash job orchestration
│   └── config/
├── gen/                       # Generated client from versioned API
├── go.mod
├── go.sum
└── README.md
```

The Go server is an **optional control-plane client** of the Rust engine. It owns user-facing HTTP concerns, but **never** performs kinematics, schedules step pulses, directly flashes hardware, or becomes the authoritative robot state machine. Firmware build/flash requests are delegated to the restricted Rust builder, with explicit authorization and safe machine state checks.

### 11.3 `sindrava` — Vite + React user interface

```text
sindrava/
├── src/
│   ├── app/
│   ├── components/
│   ├── features/
│   │   ├── dashboard/
│   │   ├── robots/
│   │   ├── composer/
│   │   ├── firmware/
│   │   ├── plugins/
│   │   ├── motion/
│   │   └── diagnostics/
│   ├── api/                   # Go API client
│   ├── hooks/
│   └── types/
├── public/
├── package.json
├── vite.config.ts
└── README.md
```

The frontend is a replaceable client, not a required runtime dependency. Production static assets may be served by the Go server, without merging repositories.

### 11.4 API ownership and independent operation

```text
sindrava (React/Vite)
       │ REST + WebSocket
       ▼
ratavyr (Go)
       │ versioned gRPC / local Unix socket
       ▼
motioncore (Rust engine + plugin runtime + firmware builder)
       │ versioned binary protocol over USB / UART / CAN
       ▼
MCU firmware (built from motioncore source)
```

- **Engine API contract:** Published and versioned by `motioncore`; `ratavyr` consumes generated client bindings from a pinned contract release. No independently edited duplicate `.proto` files.
- **MCU protocol:** Defined and versioned inside `motioncore`; shared by host, simulator and firmware.
- **CLI/headless operation:** `motioncore` must run without Go or React. Proposed commands include `motioncore start`, `motioncore mcu list`, and `motioncore firmware build`.
- **Compatibility:** Releases document supported engine/server API versions, firmware protocol versions, and plugin API versions. Incompatible peers fail safely.
- **Deployment:** Three independent build artifacts: Rust engine and tools, Go server binary, and Vite static bundle. They may run on one Linux host.

## 12. Testing and Validation

- Unit tests: kinematics, trajectory constraints, graph resolution, manifest validation.
- Protocol tests: framing, CRC failures, replay/sequence behavior, version mismatches.
- Property/fuzz tests: parsers, plugin inputs, invalid numeric trajectories.
- Simulation tests: virtual MCU executes the same commands as hardware.
- Fault injection: unplug MCU, stop heartbeat, plugin crash, buffer underrun, encoder disagreement.
- Hardware-in-the-loop: one motor, limit switch, encoder, E-stop/drive-disable verification.
- Timing benchmarks: step jitter, maximum sustainable step rate, clock drift, queue latency.
- Build reproducibility: same pinned inputs produce auditable artifacts.

Performance limits must be measured on selected hardware, not promised in advance.

## 13. Risks and Mitigations

| Risk | Priority | Mitigation |
|---|---|---|
| Unsafe plugin behavior | Critical | Capability isolation, trusted motion gate, independent hardware safety |
| MCU timing jitter | High | Hardware timers, bounded interrupts, oscilloscope/logic-analyzer tests |
| Multi-MCU synchronization | High | Defer to later phase; define timestamp model early |
| Plugin API churn | High | Versioned schemas, reference plugins, compatibility tests |
| Board/firmware fragmentation | High | One board for MVP, board metadata and capability negotiation |
| Firmware flash failure | Medium/High | Recovery procedure, artifact verification, known-good firmware |
| Web/engine state mismatch | Medium | Single Rust state authority, sequence-numbered telemetry |
| Scope explosion | High | Single-axis MVP before full 6-DOF and Robot Composer polish |

## 14. Implementation Roadmap

### Phase 0 — Architecture contracts
- Specify robot state machine, units, motion ownership, plugin manifests, and protocol messages.
- Define simulator and hardware acceptance tests.

### Phase 1 — Virtual MCU + Rust host
- Shared protocol crate, framed transport, simulated MCU.
- Device handshake, clock sync prototype, command queue and fault telemetry.

### Phase 2 — First physical axis
- Rust firmware on one selected STM32 board.
- Step/dir, enable, limit switch, watchdog and safe fault behavior.
- Validate with instrumentation and no-load test fixture.

### Phase 3 — Plugin SDK and device graph
- WASM plugin lifecycle, capability resolution, example actuator and robot plugins.
- Configuration validation and dependency handling.

### Phase 4 — Motion engine
- Joint-space trajectory generation, trusted validation, homing, stop behavior.
- Virtual/physical parity tests.

### Phase 5 — Go API + React dashboard
- Robot status, jog/homing, plugin management, telemetry and 3D visualization.

### Phase 6 — Firmware builder
- Board/module selection, reproducible compile, flash, verification, logs.

### Phase 7 — Multi-axis / AR4-MK5
- Verify actual controller electronics, pinout, gearing, limits and firmware support.
- Progress from one joint to synchronized multi-joint motion; validate safety before full-arm testing.

### Later releases
- Additional MCU families, multi-MCU sync, visual Robot Composer, richer web plugins, collision-aware planning, advanced calibration, plugin distribution.
- Separate **MotionCore ROS** and **MotionCore AI Lab** projects integrate through stable MotionCore APIs.

## 15. v0.1 Acceptance Criteria

A user can:
1. Install and start the Rust engine independently; optionally connect the Go API and React UI.
2. Load a versioned robot/actuator plugin.
3. Configure one axis through the web UI.
4. Connect a virtual MCU and issue validated jog/home/stop commands.
5. Select a supported STM32 board, compile and flash firmware.
6. Connect real hardware and confirm firmware capabilities.
7. Move a single axis safely with endstop/watchdog support.
8. Inspect position, queue state, communication health and faults.
9. Disconnect the MCU or crash a plugin and observe defined fault behavior without uncontrolled continued motion.

## 16. Open Design Decisions

- Exact first STM32 board and driver hardware.
- Initial actuator types (step/dir only versus servo PWM).
- Whether first host plugins use WASM Component Model immediately or a simpler versioned out-of-process API.
- Robot configuration schema (TOML vs YAML) and migration policy.
- First-release homing strategies and calibration persistence.
- Real-time performance targets after baseline hardware measurements.
- Firmware update recovery and board-specific flashing support.
- Licensing and governance for third-party plugins.

## 17. Guiding Principles

**Plugin-first, but not plugin-controlled safety. Firmware-buildable, but not arbitrary-hardware magic. Hardware-independent at the interface, but hardware-validated in practice.**

The first meaningful demonstration is: **configure a plugin → simulate one axis → build and flash Rust MCU firmware → move the physical axis via the Rust CLI or Go + React → verify telemetry and safe fault handling**.
