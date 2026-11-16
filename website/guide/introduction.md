# Introduction

MotionCore lets you configure, build firmware for, connect, monitor and control robotic axes without rewriting the motion-control stack for every robot. You describe the robot in a TOML file; MotionCore plans the motion on the host, streams timestamped segments to a microcontroller, and the microcontroller produces the step pulses.

It runs independently of ROS. The first physical reference robot is the Annin Robotics AR4-MK5, but nothing in the architecture depends on that model.

## The three repositories

MotionCore is split into three independently versioned repositories. Only the first is required.

| Repository | Language | Role | Required? | Docs |
|---|---|---|---|---|
| [**motioncore**](https://github.com/bzy-logics/motioncore) | Rust | Engine (the state authority), MCU firmware, simulator, plugins, firmware builder, `motioncore` CLI, engine API contract | yes | this site |
| [**ratavyr**](https://github.com/bzy-logics/ratavyr) | Go | REST/WebSocket gateway: authentication, roles, audit log, firmware job orchestration | optional | [ratavyr.bzy-logics.com](https://ratavyr.bzy-logics.com) |
| [**sindrava**](https://github.com/bzy-logics/sindrava) | TypeScript (Vite + React + Three.js) | Browser operator UI | optional | [sindrava.bzy-logics.com](https://sindrava.bzy-logics.com) |

```text
sindrava ──REST/WS──▶ ratavyr ──gRPC (engine-api v1)──▶ motioncore engine ──COBS+CRC frames──▶ MCU firmware
                                                              ▲                                (board or simulator)
                                motioncore CLI ──gRPC─────────┘
```

The rule that holds everything together: **the engine is the only owner of robot state.** The gateway and the UI relay commands and display telemetry; they never plan motion, invent state, or talk to hardware directly. The MCU, in turn, independently enforces its own safety limits even if the host misbehaves.

## What works today

| Area | Status |
|---|---|
| Engine, motion planning, safety state machine | Implemented, tested end-to-end in simulation |
| Virtual MCU (in-process and over TCP) | Implemented, runs the real firmware runtime |
| `motioncore` CLI | Implemented |
| Firmware builder (configure / build / flash / verify) | Implemented |
| STM32F411 "Black Pill" firmware (1 axis) | Compiles, host-tested, **not validated on hardware** |
| RP2040 Pico firmware (6 axes, PIO) | Compiles, host-tested, **not validated on hardware** |
| Builtin plugins | Implemented |
| External WASM plugins | Manifests validated; **execution not yet implemented** |
| ratavyr gateway (REST, WebSocket, roles, audit) | Implemented |
| sindrava UI | Implemented; Robot Composer is read-only |
| Coordinated multi-axis motion | **Not yet** — one axis is commanded at a time |
| Multiple MCUs per robot | **Not yet** — one MCU per robot |

See [Roadmap & status](/internals/roadmap) for the full list.

## Where to go next

- **Try it now, no hardware:** [Quick start](/guide/quick-start)
- **Add the web UI:** [Running the full stack](/guide/full-stack)
- **Understand the design:** [How it fits together](/guide/concepts) and [Safety model](/guide/safety)
- **Describe your robot:** [Robot configuration](/guide/configuration)
- **Connect a board:** [Hardware overview](/hardware/)

::: danger Safety
MotionCore is not a certified safety system. A software emergency stop **complements** a hardware E-stop circuit; it can never replace one. Any physical setup must have an independent E-stop that removes driver power.
:::
