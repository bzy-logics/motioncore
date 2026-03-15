# MotionCore

A plugin-first, hardware-independent robotics motion-control
runtime: **Rust** engine, MCU firmware, plugins and firmware builder.

This is the `motioncore` repository — one of three:

| Repository | Role | Required? |
|---|---|---|
| **motioncore** (this) | Engine (state authority), firmware, simulator, plugins, builder, `motioncore` CLI, engine API contract | yes |
| [ratavyr](../ratavyr) | Go REST/WebSocket gateway, auth, firmware job orchestration | optional |
| [sindrava](../sindrava) | Vite + React + Three.js dashboard | optional |

> **Status: v0.2 development.** Runs end-to-end in simulation. The STM32F411
> firmware compiles but has **not** been validated on hardware. Nothing here
> is a certified safety system — always use an independent hardware E-stop
> that removes driver power.

```text
sindrava ──REST/WS──▶ ratavyr ──gRPC (engine-api v1)──▶ motioncore engine ──COBS+CRC frames──▶ MCU firmware
                                                                          ▲                                     (boards/* or
                                         motioncore CLI ──gRPC────────────┘                                      simulator)
```

## Quick start (headless, simulation)

Requirements: Rust stable (see `rust-toolchain.toml`).

```bash
cargo build
target/debug/motioncore start --config robot-profiles/single-axis-virtual.toml   # terminal 1

target/debug/motioncore status                                                   # terminal 2
target/debug/motioncore enable
target/debug/motioncore home j1
target/debug/motioncore move j1 -90
target/debug/motioncore jog j1 5
target/debug/motioncore watch          # live telemetry + log
target/debug/motioncore estop
```

The engine listens on `unix:///run/motioncore/engine.sock` on Linux and
`tcp://127.0.0.1:7600` elsewhere (`--listen`, or `MOTIONCORE_ENGINE_ADDR` for clients).

With the optional UI: run `ratavyr` (`MOTIONCORE_ENGINE_ADDR` pointing at
the engine) and open `sindrava` (see their READMEs).

## Layout

```text
crates/
  core/              engine runtime: state authority, motion pipeline, firmware jobs, gRPC services
  cli/               `motioncore` binary (start, control client, simulator, firmware tools)
  engine-api/        versioned gRPC contract (proto/motioncore/engine/v1/engine.proto)
  protocol/          host ↔ MCU wire protocol (no_std; postcard + CRC-16 + COBS)
  safety/            robot state machine and fault model
  motion/            trapezoidal planning, validation, MCU segment generation
  kinematics/        FK/IK abstractions, joint-space identity solver
  device-graph/      robot configuration schema, board metadata, resolver
  plugin-sdk/        plugin contract (manifest, permissions, Plugin trait)
  plugin-runtime/    discovery, validation, dependency resolution, lifecycle
  mcu-manager/       MCU link: handshake, clock sync, heartbeats, link loss
  simulator/         virtual MCU running the real firmware runtime
  firmware-hal/      hardware traits implemented by boards and the simulator (no_std)
  firmware-stepper/  step/dir module: segments, homing, controlled stop (no_std)
  firmware-core/     MCU runtime: dispatch, watchdog, fault latching (no_std)
  firmware-builder/  restricted configure/build/flash/verify
boards/stm32f4/      STM32F411 "Black Pill" firmware (cross-compiled only)
plugins/builtin/     reference plugins, built against plugin-sdk
robot-profiles/      example robot configurations
docs/                specification, architecture, robot config schema, bring-up
examples/            CLI walkthroughs
```

`firmware-encoder` and `firmware-gpio` from the specification are not
created yet; they arrive with the first encoder/GPIO feature.

## Engine API contract

`crates/engine-api/proto/motioncore/engine/v1/engine.proto` is the single
source of truth (services `EngineService` and `FirmwareService`). Releases
are git tags `engine-api/vX.Y.Z`; `ratavyr` generates its Go
bindings from a pinned tag and verifies the file's SHA-256. Rules: v1 changes
are additive only; breaking changes get a new `motioncore.engine.v2` package.
Print the contract a binary was built with: `motioncore engine-api proto`.

## Tests

```bash
cargo test            # all host crates: protocol, firmware runtime, planner, graph, plugins, link, gRPC end-to-end
cargo clippy --all-targets
cargo build -p motioncore-board-stm32f4 --profile firmware --target thumbv7em-none-eabihf
```

## Firmware

```bash
motioncore firmware targets
motioncore firmware build  --config robot-profiles/single-axis-stm32f411.toml
motioncore firmware flash  --artifact build/firmware/main --yes        # probe-rs; refused while drives are on
motioncore firmware verify --artifact build/firmware/main --device /dev/ttyACM0
motioncore mcu list
```

Read [docs/hardware-bringup.md](docs/hardware-bringup.md) before connecting a motor.

## Documentation

- [docs/specification.md](docs/specification.md) — specification (draft v0.2)
- [docs/architecture.md](docs/architecture.md) — implementation notes and decisions
- [docs/robot-config.md](docs/robot-config.md) — robot configuration schema
- [docs/hardware-bringup.md](docs/hardware-bringup.md) — first physical axis

## License

Apache-2.0 OR MIT.
