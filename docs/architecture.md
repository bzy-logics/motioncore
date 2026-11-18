# Architecture notes (v0.2)

How the implementation maps onto [the specification](specification.md), and
the open decisions (§16) settled so far — provisionally.

## Repositories and contracts

| Boundary | Contract | Owner | Versioning |
|---|---|---|---|
| web ↔ server | REST + WebSocket (`ratavyr/docs/rest-api.md`) | ratavyr | v1 |
| server/CLI ↔ engine | gRPC `motioncore.engine.v1` (`crates/engine-api`) | motioncore | tag `engine-api/v0.MINOR.PATCH`, additive within a `0.MINOR` |
| engine ↔ MCU | binary protocol (`crates/protocol`) | motioncore | `PROTOCOL_VERSION`, checked at handshake |
| engine ↔ plugins | `crates/plugin-sdk` | motioncore | `API_VERSION`, checked per manifest |

Incompatible peers fail safely: the engine refuses an MCU with another
protocol version or missing capabilities; the server and CLI refuse an engine
whose `api_version` has a different `0.MINOR`; the plugin runtime refuses manifests with
another API version.

The engine listens on a Unix socket on Linux (`unix:///run/motioncore/engine.sock`)
and loopback TCP elsewhere. There is no authentication at this layer —
filesystem permissions / loopback are the boundary; user auth lives in the
server.

## Authority and data flow

* The **engine** (`crates/core`) is the only owner of robot state. The
  safety [`Supervisor`](../crates/safety/src/lib.rs) holds an explicit
  transition table; anything not in it is rejected.
* **Clients** (server, CLI) relay requests and telemetry. The server caches
  only the latest telemetry for new WebSocket clients.
* The **MCU** owns real-time execution and independently enforces soft
  limits, endstops, the step-rate ceiling, the communication watchdog and fault
  latching.

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

`Hal::step` is called once per due step. Boards that output pulses directly
(STM32F411, simulator) emit them there. The RP2040 records them and, in
`Hal::flush_pulses` at the end of the tick, hands them to PIO state
machines that spread them evenly over the next tick window with guaranteed
pulse and direction timing (see [rp2040.md](rp2040.md)).

The HAL is being reworked per [ADR 0002](adr/0002-hal-architecture.md)
(accepted): capability traits with no implicit defaults, window-level step
generation (`StepGen`) instead of one `step` call per pulse, and a
`safe_shutdown()` contract for panics and faults. The layering above stays
as shown.

Board crates are workspace members but excluded from `default-members`; they
are cross-compiled with the `firmware` profile (`opt-level = "s"`, LTO,
`panic = "abort"`). Linker arguments live in the board's `build.rs` so builds
from the workspace root link correctly.

## Motion pipeline

1. Command arrives (`Jog`, `MoveTo`, `Home`, `Stop`, `EmergencyStop`).
2. Supervisor check (one motion at a time; `ready` required).
3. Trapezoidal profile in axis units, validated for finite values, limits,
   velocity and acceleration.
4. Converted to constant-acceleration MCU segments: never straddling a
   profile phase (so `p(t) = v0·t + ½·a·t²` is exact), step counts from
   rounded *absolute* positions (no accumulated rounding), ≤ 20 ms each,
   timestamped on the MCU clock 100 ms ahead.
5. A feeder streams segments as queue space frees up. Segments, Stop and
   E-stop are submitted under the engine lock, so a Stop is never overtaken
   by a stale segment.
6. The MCU re-validates every segment and NACKs anything wrong.
7. When the MCU reports idle, commanded positions are re-synced from the
   MCU step counter.

Unhomed axes may only jog relatively, at 25 % speed, within one travel range
of the power-on position. Any fault or E-stop clears the homed flag.

## Firmware builds

`FirmwareService` jobs always build for the robot configuration the engine
was started with; clients choose only the MCU. Builds run `cargo --locked`
on in-repo board crates only. Flashing requires `confirm = true` and is
refused while drives may be energised. The CLI applies the same checks
(querying the engine if it is reachable). The lockfile hash is embedded in
the image and reported at handshake, so `motioncore firmware verify` proves
which build runs.

## Fault model

| Source | Detected by | Response |
|---|---|---|
| Host silent / USB unplugged | MCU watchdog | drives disabled, queue flushed, `communication_lost` latched |
| MCU silent / stream closed | `mcu-manager` (1 s silence or EOF) | engine `faulted`, reconnect attempted; reset lands in `disabled` |
| Queue ran dry mid-move | MCU | `queue_underrun`, drives disabled |
| Segment late | MCU | `segment_late`, drives disabled |
| Endstop during motion (toward switch) | MCU | `limit_hit`, no further step |
| Soft limit | engine (planning) and MCU (per step) | rejected / `soft_limit` |
| Plugin failure | plugin runtime | plugin + dependents `failed`; robot `plugin_failed` if required |
| Operator E-stop | engine + MCU | latched `emergency_stopped`, drives disabled |

Fault reset never resumes motion. The software E-stop **complements** a
hardware E-stop circuit; it cannot replace one.

## Decisions taken (spec §16)

| Open decision | Choice | Revisit when |
|---|---|---|
| First board | WeAct STM32F411CEU6 "Black Pill" (provisional) | after bench tests |
| Second board | Raspberry Pi Pico (RP2040), PIO step output, 6 channels (provisional) | after bench tests |
| Actuator types | step/dir only | servo PWM module |
| Host plugins | builtin (compiled-in, SDK-based); WASM manifests validated, execution deferred | Wasmtime integration |
| Config format | TOML, `schema = 1`, unknown keys rejected | first migration |
| Homing | single-switch search, position assigned at trigger | back-off + slow second touch |
| Go ↔ Rust | gRPC (tonic / grpc-go), proto compiled with `protox`/`buf` (no `protoc`) | — |
| Flashing | per board: `probe-rs` over SWD (STM32F411), UF2 to the BOOTSEL drive (RP2040) | DFU for STM32 |

Later architecture decisions are recorded as ADRs in [adr/](adr/README.md).

## Known limitations

* One MCU per robot; telemetry has one MCU block. Moves are joint-space
  and Cartesian-line moves: single-axis moves, coordinated joint moves
  (`MoveJ`), blended joint paths (`MoveJPath`) and Cartesian lines (`MoveL`,
  serial chains only); no arcs and no blending between separate commands.
* "Measured" position is the MCU step counter — no encoder feedback yet.
* Serial transport, flashing and the STM32F411 and RP2040 firmware are
  untested on hardware.
* Virtual MCU lateness tolerance is 40 ms because desktop OS timers are
  coarse (~15 ms on Windows); hardware uses 5 ms.
