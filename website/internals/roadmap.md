# Roadmap & status

MotionCore is at **v0.2 (development)**. The full design is in the [specification](https://github.com/bzy-logics/motioncore/blob/main/docs/specification.md) (draft v0.2).

## Phases

| Phase | Scope | Status |
|---|---|---|
| 0 — Architecture contracts | State machine, units, motion ownership, plugin manifests, protocol messages | Done |
| 1 — Virtual MCU + Rust host | Protocol crate, framed transport, simulated MCU, handshake, clock sync, queue, fault telemetry | Done |
| 2 — First physical axis | STM32 firmware: step/dir, enable, limit switch, watchdog, safe faults; validated with instruments | Firmware done; **hardware validation pending** |
| 3 — Plugin SDK and device graph | Plugin lifecycle, capability resolution, example plugins, config validation | Done for builtin plugins; **WASM execution pending** |
| 4 — Motion engine | Joint-space trajectories, trusted validation, homing, stop behaviour, virtual/physical parity | Single-axis moves and coordinated joint moves (MoveJ) done; Cartesian moves pending |
| 5 — Go API + React dashboard | Status, jog/homing, plugins, telemetry, 3D view | Done (ratavyr, sindrava) |
| 6 — Firmware builder | Board selection, reproducible compile, flash, verify, logs | Done |
| 7 — Multi-axis / AR4-MK5 | Real controller electronics, pinout, gearing; one joint → synchronised multi-joint | RP2040 six-channel firmware done; not started on the robot |

## v0.1 acceptance criteria

| # | A user can… | Status |
|---|---|---|
| 1 | Install and start the Rust engine independently; optionally connect the Go API and React UI | ✅ |
| 2 | Load a versioned robot/actuator plugin | ✅ builtin |
| 3 | Configure one axis through the web UI | ⚠️ UI shows config read-only; edit TOML |
| 4 | Connect a virtual MCU and issue validated jog/home/stop commands | ✅ |
| 5 | Select a supported STM32 board, compile and flash firmware | ✅ compile; flashing unverified on hardware |
| 6 | Connect real hardware and confirm firmware capabilities | ⏳ pending hardware |
| 7 | Move a single axis safely with endstop/watchdog support | ⏳ pending hardware |
| 8 | Inspect position, queue state, communication health and faults | ✅ |
| 9 | Disconnect the MCU or crash a plugin and observe defined fault behaviour | ✅ in simulation |

## Known limitations

- One MCU per robot; telemetry has one MCU block.
- Moves are joint-space only (MoveJ is straight in joint space); no Cartesian moves or blending between moves.
- Measured position is the MCU step counter; no encoder feedback.
- Serial transport, flashing and both boards' firmware are untested on hardware.
- External WASM plugins are validated but not executed.
- No configuration-editing or validation API; the Robot Composer is read-only.
- Firmware verification is CLI-only.
- Firmware job progress is polled; the event stream does not carry job updates.
- Gateway roles are per token: no user accounts, and tokens can only be rotated by restarting.
- The 3D viewer cannot draw an articulated chain; the engine does not expose link geometry.

## Later

- Additional MCU families (ESP32-S3, Teensy 4.1) after toolchain and HAL validation
- Multi-MCU synchronisation
- Editable visual Robot Composer
- Encoder and GPIO firmware modules; servo PWM
- WASM plugin execution (Wasmtime), web UI extensions
- Collision-aware planning, advanced calibration
- Homing with back-off and slow second touch
- DFU flashing for STM32
- Separate **MotionCore ROS** and **MotionCore AI Lab** projects, integrating through the stable APIs
