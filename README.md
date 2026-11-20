# MotionCore

**Website & docs: [motioncore.bzy-logics.com](https://motioncore.bzy-logics.com)** · Gateway: [ratavyr.bzy-logics.com](https://ratavyr.bzy-logics.com) · UI: [sindrava.bzy-logics.com](https://sindrava.bzy-logics.com)

A plugin-first, hardware-independent robotics motion-control system in
**Rust**: the engine, a simulator, a firmware builder, a CLI, and MCU firmware
for **STM32F411** and **RP2040** (PIO-timed step/dir, up to 6 axes).

- **One firmware core, many boards:** the protocol, segment execution,
  homing, watchdog and fault handling are shared by every MCU target and the
  simulator. A board only supplies pins, timing and transport.
- **Safety first:** the MCU enforces soft limits, endstops, the step-rate
  ceiling and a communication watchdog on its own, and latches faults that
  can only be cleared once the hazard is gone.
- **Deterministic step output:** on RP2040, PIO state machines time every
  pulse, with guaranteed pulse widths and direction setup/hold, and no catch-up
  bursts.
- **Reproducible firmware:** builds are pinned by a lockfile, artifacts are
  checksummed, and the running build is verified over the wire.

This is the `motioncore` repository — one of three:

| Repository | Role | Required? |
|---|---|---|
| **motioncore** (this) | Engine (state authority), firmware, simulator, plugins, builder, `motioncore` CLI, engine API contract | yes |
| [ratavyr](https://github.com/bzy-logics/ratavyr) | Go REST/WebSocket gateway, auth, firmware job orchestration | optional |
| [sindrava](https://github.com/bzy-logics/sindrava) | Vite + React + Three.js dashboard | optional |

> **Status: v0.3.0** (see [CHANGELOG.md](CHANGELOG.md)); v0.4 development on `main`. Runs end-to-end in simulation. The STM32F411
> and RP2040 firmware compile and are host-tested, but have **not** been
> validated on hardware. Nothing here is a certified safety system — always
> use an independent hardware E-stop that removes driver power.

```text
sindrava ──REST/WS──▶ ratavyr ──gRPC (engine-api v1)──▶ motioncore engine ──COBS+CRC frames──▶ MCU firmware
                                                                          ▲                                     (boards/* or
                                         motioncore CLI ──gRPC────────────┘                                      simulator)
```

## Supported hardware

| board | step channels | step output | flashing | status |
|---|---|---|---|---|
| `virtual` | 6 | simulated (shared firmware core) | – | host-tested |
| `stm32f411-blackpill` | 1 | GPIO, 20 kHz timer interrupt | probe-rs (SWD) | provisional, unverified on hardware |
| `rp2040-pico` | 6 | PIO, one state machine per axis | UF2 (BOOTSEL) | provisional, unverified on hardware |

Physical boards use external step/dir drivers and 3.3 V logic. See
[docs/rp2040.md](docs/rp2040.md) and
[docs/hardware-bringup.md](docs/hardware-bringup.md) for wiring and the
hardware validation procedure.

## Quick start (headless, simulation)

Requirements: Rust stable (see `rust-toolchain.toml`; it also installs the
`thumbv7em-none-eabihf` and `thumbv6m-none-eabi` targets for firmware builds).

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

For a multi-axis machine with kinematic geometry (a two-link arm on a linear
rail, drawn as a connected robot in the web UI), use
`--config robot-profiles/rail-arm-virtual.toml`, or
`robot-profiles/dummy-arm-virtual.toml` for a simulated six-axis desktop arm.
The six-axis profile also gives its links a thickness, so once homed the
engine refuses moves that would make the arm collide with itself, and a
home-all plan (`motioncore home-all`) that homes the wrist joints together,
then the arm joints together. Coordinated joint moves run all six joints
together: `motioncore movej j1=60 j2=30 j3=120 j4=90 j5=-60 j6=180`;
`motioncore movej-path` runs through several poses without stopping, and
`motioncore movel X Y Z` moves the tool in a straight line.

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
  kinematics/        KinematicModel (one FK), serial-chain IK and Cartesian lines, capsule collision model
  device-graph/      robot configuration schema, board metadata, resolver
  plugin-sdk/        plugin contract (manifest, permissions, Plugin trait)
  plugin-runtime/    discovery, validation, dependency resolution, lifecycle
  mcu-manager/       MCU link: handshake, clock sync, heartbeats, link loss
  simulator/         virtual MCU running the real firmware runtime
  firmware-hal/      hardware traits implemented by boards and the simulator (no_std)
  firmware-stepper/  step/dir module: segments, homing, controlled stop (no_std)
  firmware-core/     MCU runtime: dispatch, watchdog, fault latching (no_std)
  firmware-pio-step/ RP2040 PIO STEP/DIR program, command encoding, step buffering (no_std)
  firmware-builder/  restricted configure/build/flash/verify, ELF -> UF2
boards/stm32f4/      STM32F411 "Black Pill" firmware (cross-compiled only)
boards/rp2040/       Raspberry Pi Pico firmware, PIO step output (cross-compiled only)
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
are git tags `engine-api/v0.MINOR.PATCH`; `ratavyr` generates its Go
bindings from a pinned tag and verifies the file's SHA-256. While pre-1.0,
PATCH releases are additive only; breaking changes and major features bump
MINOR, and clients accept an engine with the same `0.MINOR`.
Print the contract a binary was built with: `motioncore engine-api proto`.

## Tests

```bash
cargo test            # all host crates: protocol, firmware runtime, planner, graph, plugins, link, gRPC end-to-end
cargo clippy --all-targets
cargo build -p motioncore-board-stm32f4 --profile firmware --target thumbv7em-none-eabihf
cargo build -p motioncore-board-rp2040  --profile firmware --target thumbv6m-none-eabi
```

## Firmware

```bash
motioncore firmware targets
motioncore firmware build  --config robot-profiles/single-axis-stm32f411.toml
motioncore firmware flash  --artifact build/firmware/main --yes        # probe-rs; refused while drives are on
motioncore firmware verify --artifact build/firmware/main --device /dev/ttyACM0

motioncore firmware build  --board rp2040-pico                         # reference profile, writes firmware.uf2
motioncore firmware flash  --board rp2040-pico --yes                   # UF2 to the BOOTSEL drive
motioncore firmware verify --board rp2040-pico --device /dev/ttyACM0
motioncore mcu list
```

Flashing is refused while a reachable engine reports enabled drives, and
only `verify` confirms which build is running. Read [docs/hardware-bringup.md](docs/hardware-bringup.md) (STM32F411) or
[docs/rp2040.md](docs/rp2040.md) (RP2040) before connecting a motor.

## Documentation

- [docs/specification.md](docs/specification.md) — specification (draft v0.2)
- [docs/architecture.md](docs/architecture.md) — implementation notes and decisions
- [docs/robot-config.md](docs/robot-config.md) — robot configuration schema
- [docs/hardware-bringup.md](docs/hardware-bringup.md) — first physical axis (STM32F411)
- [docs/rp2040.md](docs/rp2040.md) — RP2040 firmware: PIO design, pin map, flashing, validation

## License

Copyright (C) 2026 bzy-logics

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, version 3. It is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
or FITNESS FOR A PARTICULAR PURPOSE. See [LICENSE](LICENSE) for the full text
(SPDX: `GPL-3.0-only`).
