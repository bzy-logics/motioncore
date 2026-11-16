# MCU wire protocol

The host ↔ MCU protocol lives in `crates/protocol`. The crate is `no_std`, allocation-free and `forbid(unsafe_code)`, so the **same** message definitions and framing code run on the firmware, the virtual MCU and the host.

## Framing

```text
COBS( postcard(message) ‖ crc16_le ) ‖ 0x00
```

| Layer | Purpose |
|---|---|
| [postcard](https://docs.rs/postcard) | Compact, versionable binary encoding of the message |
| CRC-16/CCITT-FALSE | Detects corrupted frames. Polynomial `0x1021`, init `0xFFFF`, no reflection, no xorout; check value `0x29B1` for `"123456789"`. Appended little-endian |
| COBS | Removes all zero bytes, so `0x00` can delimit frames and the receiver can resynchronise after garbage |

| Constant | Value |
|---|---|
| `PROTOCOL_VERSION` | `1` — bumped on any incompatible change |
| `MAX_PAYLOAD` | 224 bytes (message + CRC, before COBS) |
| `MAX_AXES` | 8 per MCU |

The streaming decoder yields one result per delimiter: a message, or one of `TooLarge`, `Cobs`, `Crc`, `Decode` (intact frame that does not decode, e.g. a version mismatch) or `Truncated`. CRC failures are counted in telemetry as `crc_errors`.

Transports: USB CDC-ACM on the boards, TCP for the out-of-process simulator, and an in-process duplex for the in-process virtual MCU.

## Host → MCU

Every host frame is `HostFrame { seq: u32, msg: HostMsg }`. `seq` increases monotonically and is echoed in `Ack` / `Nack`.

| Message | Purpose | Idempotent |
|---|---|---|
| `Identify` | Request `Identity` | yes |
| `ClockSync { host_time_us }` | Clock-sync probe → `ClockSyncReply` | yes |
| `Heartbeat` | Keep-alive for the communication watchdog | yes |
| `SetWatchdog { timeout_ms }` | Configure the watchdog | yes |
| `ConfigureAxis(AxisConfig)` | Per-axis limits, max rate, stop deceleration, inversions. Rejected while the axis moves | yes |
| `Enable { axis }` / `Disable { axis }` | Drive enable; disable is immediate, no deceleration | |
| `QueueSegment(Segment)` | Append a motion segment | **no** — never blindly retransmitted |
| `Home(HomeParams)` | Start homing → `HomingDone` | |
| `SetPosition { axis, steps }` | Overwrite the position of an idle axis | |
| `Stop` | Discard queue, decelerate at `stop_decel`; drives stay on | |
| `EmergencyStop` | Halt and disable all drives immediately; latch `EmergencyStop` | |
| `ResetFault` | Clear a latched fault. Never resumes motion; drives stay off | |
| `QueryStatus` | Request an immediate `Status` | yes |

`AxisConfig` (all in MCU units): `axis`, `min_steps`, `max_steps`, `max_rate` (hard step-rate ceiling, steps/s), `stop_decel` (steps/s²), `invert_dir`, `invert_endstop`. The engine sets `max_rate` to 105 % of `max_velocity × steps_per_unit`.

`Segment` and `HomeParams` are described in [Motion pipeline](/internals/motion-pipeline).

## MCU → host

`McuFrame { seq: u32, msg: McuMsg }`

| Message | Meaning |
|---|---|
| `Identity` | `protocol_version`, `firmware_version` (major, minor, patch), `target`, `board`, `build_hash` (first 4 bytes), `capabilities` (bitmask), `axes`, `queue_capacity` |
| `ClockSyncReply { host_time_us, mcu_time_us }` | Clock-sync answer |
| `Ack { seq }` | Command accepted |
| `Nack { seq, reason }` | Command refused: `UnknownAxis`, `QueueFull`, `InvalidState`, `InvalidParams`, `SoftLimit`, `Faulted`, `Busy` |
| `Status` | `mcu_time_us`, `fault`, `queue_depth`, and per axis: `position_steps`, `velocity`, `enabled`, `moving`, `homing`, `endstop` |
| `Fault { code, axis }` | A fault latched: `CommunicationLost`, `QueueUnderrun`, `SegmentLate`, `LimitHit`, `SoftLimit`, `HomingFailed`, `EmergencyStop`, `StepRateExceeded` |
| `HomingDone { axis, position_steps }` | Homing finished |

### Capability bits

| Bit | Name |
|---|---|
| `1 << 0` | `stepper.step_dir` |
| `1 << 1` | `endstop` |
| `1 << 2` | `watchdog` |
| `1 << 3` | `encoder.quadrature` |
| `1 << 4` | `gpio.input` |
| `1 << 5` | `gpio.output` |
| `1 << 6` | `servo.pwm` |

## Connection sequence

```text
host                                         MCU
 │── Identify ───────────────────────────────▶│
 │◀────────────────────────────── Identity ───│  check protocol version, board id, capabilities
 │── ClockSync × 8 ──────────────────────────▶│
 │◀────────────────────── ClockSyncReply × 8 ─│  keep the sample with the lowest RTT
 │── SetWatchdog ────────────────────────────▶│
 │── ConfigureAxis, SetPosition 0 (per axis) ▶│
 │◀─────────────────────────────────── Ack ───│
 │── Heartbeat every 100 ms ────────────────▶ │  engine: initializing → disabled
```

The engine refuses an MCU whose protocol version differs, whose board id differs from the configuration, or that lacks a capability the configuration requires.

## Retry rules

- Idempotent configuration messages may be retried.
- **Motion segments are never retransmitted.** If an ACK times out (500 ms), the timeout surfaces as an error and the engine stops safely, rather than risk a duplicated segment.
