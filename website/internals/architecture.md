# Architecture

This section is for contributors. It describes how the implementation is organised and the decisions behind it. For the user-level picture, start with [How it fits together](/guide/concepts).

## Workspace layout (motioncore)

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
  firmware-builder/  restricted configure/build/flash/verify, ELF → UF2
boards/stm32f4/      STM32F411 "Black Pill" firmware (cross-compiled only)
boards/rp2040/       Raspberry Pi Pico firmware, PIO step output (cross-compiled only)
plugins/builtin/     reference plugins, built against plugin-sdk
robot-profiles/      example robot configurations
docs/                specification, architecture, robot config schema, bring-up
examples/            CLI walkthroughs
```

`firmware-encoder` and `firmware-gpio` from the specification don't exist yet; they arrive with the first encoder or GPIO feature.

The gateway and UI code layouts are documented on their own sites: [ratavyr](https://ratavyr.bzy-logics.com/development/layout), [sindrava](https://sindrava.bzy-logics.com/development/layout).

## Authority and data flow

- The **engine** (`crates/core`) is the only owner of robot state. The safety `Supervisor` holds an explicit transition table; anything not in it is rejected. See [Robot states](/guide/robot-states).
- **Clients** (ratavyr, CLI) relay requests and telemetry. ratavyr caches only the latest telemetry for new WebSocket clients.
- The **MCU** owns real-time execution and independently enforces soft limits, endstops, the step-rate ceiling, the communication watchdog and fault latching.

## Firmware layering

```text
boards/stm32f4 ──┐   boards/rp2040 ──┐        crates/simulator ──┐
  (TIM3 tick,    │     (TIMER tick,   │          (tokio tick,     │ impl Hal
   USB CDC,      │      USB CDC, WDT, │           duplex/TCP)     │
   IWDG)         ▼      PIO step out) ▼                           ▼
              firmware-core  ◀────── identical runtime ────── firmware-core
                 │  dispatch, watchdog, faults, status
                 ▼
              firmware-stepper   per-channel queue, segment execution, homing, stop
                 │
              firmware-hal       Hal / Clock traits (+ flush_pulses)
                 │
              firmware-pio-step  RP2040 only: PIO program + per-tick step buffering
```

`Hal::step` is called once per due step. Boards that output pulses directly (STM32F411, simulator) emit them there. The RP2040 records them and, in `Hal::flush_pulses` at the end of the tick, hands them to PIO state machines that spread them evenly over the next tick window. `flush_pulses` defaults to a no-op, so other boards are unaffected. See [RP2040](/hardware/rp2040#architecture).

Board crates are workspace members but excluded from `default-members`. They are cross-compiled with the `firmware` profile (`opt-level = "s"`, LTO, `panic = "abort"`). Linker arguments live in each board's `build.rs`, so builds from the workspace root link correctly.

## The MCU link

`crates/mcu-manager` owns one framed byte stream to one MCU and provides:

- request/ACK correlation by sequence number, with a 500 ms ACK timeout;
- the identification handshake with protocol-version and board check;
- NTP-style clock synchronisation (best of N probes by round-trip time);
- heartbeats every 100 ms, feeding the MCU's watchdog;
- link-loss detection: stream closed, or MCU silent for 1 s;
- link statistics for diagnostics.

Outbound traffic uses two **bounded** lanes. The priority lane carries only messages that are safe to send ahead of queued traffic — emergency stop, heartbeat, status query, identify, clock sync. Everything else keeps its submission order on the normal lane. A full lane is reported as backpressure instead of growing without limit, and each transport write is capped at 256 bytes so priority frames never wait behind a large batch.

Non-idempotent commands (motion segments) are **never retransmitted**. A timeout surfaces as an error and the engine decides to stop safely.

See [MCU wire protocol](/internals/mcu-protocol).

## Firmware builds

`FirmwareService` jobs always build for the robot configuration the engine was started with; clients choose only the MCU. Builds run `cargo --locked` on in-repo board crates only. Flashing requires `confirm = true` and is refused while drives may be energised; the CLI applies the same checks, querying the engine if it is reachable. The lockfile hash is embedded in the image and reported at handshake, so `motioncore firmware verify` proves which build runs.

## Fault model

| Source | Detected by | Response |
|---|---|---|
| Host silent / USB unplugged | MCU watchdog | Drives disabled, queue flushed, `communication_lost` latched |
| MCU silent / stream closed | `mcu-manager` (1 s silence or EOF) | Engine `faulted`, reconnect attempted; reset lands in `disabled` |
| Queue ran dry mid-move | MCU | `queue_underrun`, drives disabled |
| Segment late | MCU | `segment_late`, drives disabled |
| Endstop during motion (toward switch) | MCU | `limit_hit`, no further step |
| Soft limit | engine (planning) and MCU (per step) | Rejected / `soft_limit` |
| Plugin failure | plugin runtime | Plugin + dependents `failed`; robot `plugin_failed` if required |
| Operator E-stop | engine + MCU | Latched `emergency_stopped`, drives disabled |

Fault reset never resumes motion.

## Decisions taken

The specification left several decisions open. They are settled provisionally:

| Open decision | Choice | Revisit when |
|---|---|---|
| First board | WeAct STM32F411CEU6 "Black Pill" | After bench tests |
| Second board | Raspberry Pi Pico (RP2040), PIO step output, 6 channels | After bench tests |
| Actuator types | Step/dir only | Servo PWM module |
| Host plugins | Builtin (compiled-in, SDK-based); WASM manifests validated, execution deferred | Wasmtime integration |
| Config format | TOML, `schema = 1`, unknown keys rejected | First migration |
| Homing | Single-switch search, position assigned at trigger | Back-off + slow second touch |
| Go ↔ Rust | gRPC (tonic / grpc-go); proto compiled with `protox` / `buf` (no `protoc`) | — |
| Flashing | Per board: `probe-rs` over SWD (STM32F411), UF2 to the BOOTSEL drive (RP2040) | DFU for STM32 |
