# How it fits together

## Who owns what

MotionCore has a strict division of responsibility. Each layer has one job, and no layer does another's.

| Component | Owns | Never does |
|---|---|---|
| **MCU firmware** | Real-time step generation; its own enforcement of soft limits, endstops, the step-rate ceiling and the communication watchdog; fault latching | Plan motion, trust the host blindly |
| **Engine** (`motioncore`) | Robot state, the safety state machine, motion planning and validation, the MCU link, firmware build jobs | Emit pulses itself |
| **Gateway** (`ratavyr`) | HTTP concerns: authentication, roles, rate limiting, audit log, WebSocket fan-out | Keep a robot state machine, plan motion, flash hardware directly |
| **UI** (`sindrava`) | Presentation | Synthesise robot state; its button enablement is a convenience only |
| **Plugins** | Robot profiles, actuator types, kinematics; motion *proposals* | Command hardware |

The gateway caches only the latest telemetry frame, so a newly connected WebSocket client gets something to show immediately. Everything else is relayed from the engine.

## The layers

```text
Browser: sindrava (React + Three.js)
            │  REST for commands and config, WebSocket for live state
            ▼
       ratavyr (Go)               auth · roles · audit · rate limit
            │  gRPC motioncore.engine.v1, Unix socket or loopback
            ▼
   motioncore engine (Rust)
   ├── plugin runtime           discover → validate → resolve → activate
   ├── device graph             TOML config → validated robot model
   ├── safety supervisor        explicit state-transition table
   ├── motion planner           trapezoidal profiles → MCU segments
   └── MCU manager              handshake · clock sync · heartbeats · link loss
            │  COBS + CRC-16 framed postcard messages over USB CDC / TCP / in-process
            ▼
   MCU firmware (STM32F411, RP2040, or the virtual MCU)
   ├── firmware-core            dispatch · watchdog · fault latching · status
   ├── firmware-stepper         segment execution · homing · controlled stop · limits
   └── board crate              pins · tick timer · transport
            │  STEP / DIR / ENABLE
            ▼
   External step/dir drivers → motors
```

## Key ideas

### The engine is the state authority

There is exactly one robot state machine, in the engine's safety `Supervisor`. It holds an explicit transition table; anything not in the table is rejected. Clients learn the state only from telemetry. See [Robot states](/guide/robot-states).

### Motion is planned on the host, executed on the MCU

When you `move j1 -90`, the engine plans a trapezoidal velocity profile, validates it against limits, and cuts it into short constant-acceleration **segments** (≤ 20 ms each), timestamped on the MCU's clock 100 ms in the future. A feeder streams segments as queue space frees up. The MCU re-validates every segment and executes it on its own timer. Details: [Motion pipeline](/internals/motion-pipeline).

### The MCU defends itself

The firmware does not trust the host. It enforces soft limits before every step, checks endstops, caps the step rate, and disables all drives if it hears nothing from the host within the watchdog timeout. If the queue runs dry mid-move or a segment arrives late, it faults instead of guessing.

### Simulation runs the real firmware

The virtual MCU is not a mock. It runs the same `firmware-core` and `firmware-stepper` code as the boards, behind a software implementation of the hardware traits. Only the board crate differs.

### Contracts are versioned and checked

Every boundary has a versioned contract, and incompatible peers refuse each other rather than half-working:

| Boundary | Contract | Check |
|---|---|---|
| UI ↔ gateway | REST/WebSocket API v1 | — |
| gateway/CLI ↔ engine | gRPC `motioncore.engine.v1` | `api_version` major must be 1 |
| engine ↔ MCU | binary protocol | `PROTOCOL_VERSION` must match; board id must match config |
| engine ↔ plugins | plugin SDK | manifest `api_version` must match |

See [Contracts & versioning](/internals/versioning).

## Glossary

| Term | Meaning |
|---|---|
| **Axis** | One motor-driven degree of freedom, identified by an id such as `j1`. Revolute (degrees) or prismatic (millimetres). |
| **Channel** | A step/dir output on an MCU. Each axis maps to one channel. |
| **Segment** | A constant-acceleration piece of a move for one axis, in MCU steps and MCU time. |
| **Profile** | (1) A robot configuration TOML file in `robot-profiles/`. (2) The robot-profile plugin that file names in `robot.profile`. |
| **Homing** | Driving an axis toward its endstop until it triggers, then assigning a known position. Absolute moves need it. |
| **Soft limit** | The configured `min`/`max` of an axis. Enforced by the engine at planning time and by the MCU per step. |
| **Latched fault** | A fault that stays active until explicitly reset, and that can only be reset once its cause is gone. |
| **Build hash** | Hash of the firmware lockfile, embedded in the image and reported by the MCU, proving which build is running. |
