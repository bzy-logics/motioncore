# Engine gRPC API

The engine's control API is gRPC package **`motioncore.engine.v1`**. ratavyr and the `motioncore` CLI are clients of it; you can write your own in any language with gRPC support.

- **Contract file:** [`crates/engine-api/proto/motioncore/engine/v1/engine.proto`](https://github.com/bzy-logics/motioncore/blob/main/crates/engine-api/proto/motioncore/engine/v1/engine.proto) — the single source of truth.
- **Print it from a binary:** `motioncore engine-api proto`.
- **Releases:** git tags `engine-api/vX.Y.Z`. Generate bindings from a tagged release; never copy and edit the file.

## Connecting

| Platform | Default address |
|---|---|
| Linux | `unix:///run/motioncore/engine.sock` |
| Windows, macOS | `127.0.0.1:7600` (plain TCP) |

There is **no authentication** at this layer: filesystem permissions or loopback are the boundary. Never expose the engine on a network; put [ratavyr](https://ratavyr.bzy-logics.com/guide/deployment) in front.

Clients should call `GetInfo` first and refuse to continue unless `api_version` has major version 1.

## Errors

Errors are gRPC status codes. The precise MotionCore code is in the trailing metadata key **`motioncore-error-code`**:

| `motioncore-error-code` | gRPC status |
|---|---|
| `bad_request` | `INVALID_ARGUMENT` |
| `validation_failed` | `INVALID_ARGUMENT` |
| `unknown_axis` | `NOT_FOUND` |
| `not_found` | `NOT_FOUND` |
| `not_homed` | `FAILED_PRECONDITION` |
| `invalid_state` | `FAILED_PRECONDITION` |
| `mcu_error` | `UNAVAILABLE` |
| `internal` | `INTERNAL` |

Positions, deltas and velocities are in the axis display unit (`deg` or `mm`).

## `EngineService`

| RPC | Request | Response | Notes |
|---|---|---|---|
| `GetInfo` | – | `EngineInfo` | Handshake. Check `api_version` |
| `GetStatus` | – | `Telemetry` | Current snapshot |
| `GetRobotConfig` | – | `RobotConfig` | Loaded configuration |
| `ListPlugins` | – | `ListPluginsResponse` | |
| `StreamEvents` | – | stream of `Event` | Telemetry (~20 Hz) and log entries. The first event is always a telemetry snapshot |
| `Enable` | – | `CommandResponse` | |
| `Disable` | – | `CommandResponse` | Controlled stop if moving, then drive disable |
| `Jog` | `axis`, `delta`, optional `velocity` | `CommandResponse` | Default velocity: axis maximum (25 % when unhomed) |
| `MoveTo` | `axis`, `position`, optional `velocity` | `CommandResponse` | Axis must be homed |
| `Home` | `axis` | `CommandResponse` | |
| `Stop` | – | `CommandResponse` | Controlled stop |
| `EmergencyStop` | – | `CommandResponse` | Latches `emergency_stopped` |
| `ResetFault` | – | `CommandResponse` | Never resumes motion; drives stay disabled; re-home required |
| `DebugDisconnectMcu` | – | `CommandResponse` | Only with `--enable-fault-injection`; otherwise `FAILED_PRECONDITION` |
| `DebugFailPlugin` | `id` | `CommandResponse` | Same |

Command RPCs return when the engine has **accepted** the command. Follow progress through `StreamEvents`.

## `FirmwareService`

Firmware build and flash, delegated to the restricted Rust builder. Builds always use the robot configuration the engine loaded; clients cannot supply paths, scripts or toolchains.

| RPC | Request | Response | Notes |
|---|---|---|---|
| `ListTargets` | – | `ListTargetsResponse` | Supported boards |
| `StartBuild` | `mcu` | `FirmwareJob` | `mcu` may be empty when the robot has one MCU |
| `StartFlash` | `build_job_id`, `confirm` | `FirmwareJob` | Requires `confirm = true` and drives not energised |
| `GetJob` | `id` | `FirmwareJob` | |
| `ListJobs` | – | `ListJobsResponse` | |

## Messages

### `EngineInfo`

| Field | Type | Meaning |
|---|---|---|
| `api_version` | string | Semantic version of this contract, e.g. `1.1.0` |
| `engine_version` | string | |
| `mcu_protocol_version` | uint32 | MCU wire protocol version |
| `plugin_api_version` | string | Host plugin API version |
| `fault_injection_enabled` | bool | |

### `Telemetry`

| Field | Type | Meaning |
|---|---|---|
| `seq` | uint64 | Monotonic per engine process; gaps mean dropped samples |
| `time_ms` | uint64 | Milliseconds since engine start |
| `state` | `RobotState` | |
| `fault` | optional `Fault` | Unset when no fault is latched |
| `mcu` | `McuTelemetry` | |
| `axes` | repeated `AxisTelemetry` | |

`Fault`: `code` (`FaultCode`), `message`, `axis` (empty when not axis-specific).

### `McuTelemetry`

| Field | Type | Meaning |
|---|---|---|
| `connected` | bool | |
| `transport` | string | `virtual`, `tcp` or `serial` |
| `target`, `board`, `firmware_version`, `build_hash` | string | Empty until the MCU identifies itself |
| `protocol_version` | uint32 | |
| `capabilities` | repeated string | e.g. `stepper.step_dir`, `endstop`, `watchdog` |
| `queue_capacity`, `queue_depth` | uint32 | MCU segment queue |
| `clock_offset_us` | int64 | MCU clock vs. host clock |
| `rtt_us` | uint32 | Round-trip time |
| `tx_frames`, `rx_frames`, `crc_errors` | uint64 | Link counters |
| `last_heartbeat_ms_ago` | optional uint64 | Unset while no link exists |

### `AxisTelemetry`

`id`, `homed`, `enabled`, `commanded_position`, `measured_position` (from the MCU step counter; no encoder feedback in v1.0), `velocity`, `endstop`.

### `RobotConfig`, `McuConfig`, `AxisConfig`

`RobotConfig`: `name`, `profile`, `axes`, `mcus`.
`McuConfig`: `name`, `board`, `transport`, `watchdog_ms`.
`AxisConfig`: `id`, `name`, `kind` (`JointKind`), `unit`, `min`, `max`, `max_velocity`, `max_acceleration`, optional `home_position`, `steps_per_unit`, `mcu`, `mcu_channel`, `actuator`, `step_pin`, `dir_pin`, `enable_pin`, `endstop_pin` (empty when there is no endstop), `geometry` (`AxisGeometry`, unset when the profile defines none; since 1.1.0).

### `AxisGeometry` (since 1.1.0)

URDF-style joint placement: `frame(child) = frame(parent) · Translate(origin_mm) · RPY(rotation_deg) · Joint(q about axis)`, where `frame(parent)` includes the parent's joint motion.

| Field | Type | Meaning |
|---|---|---|
| `parent` | string | Parent axis id; empty = mounted on the robot base |
| `origin_mm` | `Vec3` | Joint origin in the parent frame, mm |
| `rotation_deg` | `Vec3` | Fixed roll, pitch, yaw in degrees (about X, Y, Z, in that order) |
| `axis` | `Vec3` | Unit rotation axis (revolute) or slide direction (prismatic) |
| `tip_mm` | `Vec3` | End of this joint's link (e.g. tool flange), joint frame; unset if not configured |

`Vec3` is `x`, `y`, `z` (double). Either every axis has geometry or none does.

### `Plugin`

`id`, `version`, `api_version`, `kind` (`builtin` or `wasm`), `description`, `provides`, `requires`, `permissions`, `state` (`PluginState`), `error` (empty unless failed).

### `Event`

A `oneof`: `telemetry` (`Telemetry`) or `log` (`LogEntry`: `time_ms`, `level`, `message`).

### `FirmwareTarget`

`board`, `description`, `firmware_target` (empty for virtual), `rust_target`, `step_channels`, `max_step_rate`, `provisional`, `is_virtual`.

### `FirmwareJob`

`id`, `kind` (`JobKind`), `state` (`JobState`), `mcu`, `created_unix`, `finished_unix`, `message` (summary of the outcome), `log` (tail of the build/flash log), optional `artifact`.

`FirmwareArtifact`: `board`, `firmware_version`, `build_hash`, `image_sha256`, `toolchain`, `required_capabilities`.

## Enums

| Enum | Values (after the `UNSPECIFIED = 0` entry) |
|---|---|
| `RobotState` | `DISCONNECTED`, `INITIALIZING`, `DISABLED`, `READY`, `HOMING`, `RUNNING`, `FAULTED`, `EMERGENCY_STOPPED` |
| `FaultCode` | `COMMUNICATION_LOST`, `QUEUE_UNDERRUN`, `SEGMENT_LATE`, `LIMIT_HIT`, `SOFT_LIMIT`, `HOMING_FAILED`, `MCU_FAULT`, `PLUGIN_FAILED`, `EMERGENCY_STOP` |
| `JointKind` | `REVOLUTE`, `PRISMATIC` |
| `PluginState` | `DISCOVERED`, `VALIDATED`, `RESOLVED`, `INITIALIZED`, `ACTIVE`, `FAILED`, `DEACTIVATED` |
| `LogLevel` | `DEBUG`, `INFO`, `WARN`, `ERROR` |
| `JobKind` | `BUILD`, `FLASH` |
| `JobState` | `QUEUED`, `RUNNING`, `SUCCEEDED`, `FAILED` |

Each value is prefixed with the enum name in the proto, e.g. `ROBOT_STATE_READY`. A newer 1.x engine may add values; clients must handle unknown ones.

## Compatibility rules for v1

| Version | Change |
|---|---|
| 1.0.x | Initial contract |
| 1.1.0 | `AxisConfig.geometry` (`AxisGeometry`, `Vec3`) |

- Only additive changes: new fields, messages, RPCs and enum values.
- Field numbers are never reused; removed fields are `reserved`.
- Breaking changes require a new package, `motioncore.engine.v2`.

See [Contracts & versioning](/internals/versioning).
