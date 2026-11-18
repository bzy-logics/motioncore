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
| `collision` | `FAILED_PRECONDITION` (since 0.1.4) |
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
| `MoveJ` | `targets` (repeated `JointTarget`: `axis`, `position`), optional `speed_scale` | `CommandResponse` | Since 0.2.0. Listed axes start and finish together on a straight joint-space line; all must be homed; `speed_scale` in (0, 1], default 1. Errors: `bad_request` (no targets, axis twice), `validation_failed`, `not_homed`, `collision` |
| `HomeAll` | – | `CommandResponse` | Since 0.1.4. Homes every homeable axis following `RobotConfig.home_all`; the robot must be ready |
| `Stop` | – | `CommandResponse` | Controlled stop |
| `EmergencyStop` | – | `CommandResponse` | Latches `emergency_stopped` |
| `ResetFault` | – | `CommandResponse` | Never resumes motion; drives stay disabled; re-home required |
| `DebugDisconnectMcu` | – | `CommandResponse` | Only with `--enable-fault-injection`; otherwise `FAILED_PRECONDITION` |
| `DebugFailPlugin` | `id` | `CommandResponse` | Same |

Command RPCs return when the engine has **accepted** the command. Follow progress through `StreamEvents`.

When `RobotConfig.collision_checking` is true and every axis is homed, `Jog`, `MoveTo` and `MoveJ` are refused with `collision` if the swept move would bring two links within 2 mm of each other, or a link to the floor; the message names the links and the joint value. See [Collision checking](/guide/configuration#collision-checking).

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
| `api_version` | string | Semantic version of this contract, e.g. `0.2.0` |
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

`id`, `homed`, `enabled`, `commanded_position`, `measured_position` (from the MCU step counter; no encoder feedback yet), `velocity`, `endstop`, `homing` (true while this axis searches for its switch; since 0.1.4).

### `RobotConfig`, `McuConfig`, `AxisConfig`

`RobotConfig`: `name`, `profile`, `axes`, `mcus`, `home_all` (`HomeAllPlan`; since 0.1.4), `collision_checking` (bool: moves are checked against the link model once all axes are homed; since 0.1.4).
`McuConfig`: `name`, `board`, `transport`, `watchdog_ms`.
`AxisConfig`: `id`, `name`, `kind` (`JointKind`), `unit`, `min`, `max`, `max_velocity`, `max_acceleration`, optional `home_position`, `steps_per_unit`, `mcu`, `mcu_channel`, `actuator`, `step_pin`, `dir_pin`, `enable_pin`, `endstop_pin` (empty when there is no endstop), `geometry` (`AxisGeometry`, unset when the profile defines none; since 0.1.3).

### `AxisGeometry` (since 0.1.3)

URDF-style joint placement: `frame(child) = frame(parent) · Translate(origin_mm) · RPY(rotation_deg) · Joint(q about axis)`, where `frame(parent)` includes the parent's joint motion.

| Field | Type | Meaning |
|---|---|---|
| `parent` | string | Parent axis id; empty = mounted on the robot base |
| `origin_mm` | `Vec3` | Joint origin in the parent frame, mm |
| `rotation_deg` | `Vec3` | Fixed roll, pitch, yaw in degrees (about X, Y, Z, in that order) |
| `axis` | `Vec3` | Unit rotation axis (revolute) or slide direction (prismatic) |
| `tip_mm` | `Vec3` | End of this joint's link (e.g. tool flange), joint frame; unset if not configured |
| `radius_mm` | optional double | Link thickness for collision checking; unset if not configured (since 0.1.4) |

`Vec3` is `x`, `y`, `z` (double). Either every axis has geometry or none does.

### `HomeAllPlan` (since 0.1.4)

What `HomeAll` does: `groups` (repeated `HomeAllGroup`, each a list of `axes`) home in order; the axes of one group home at the same time. `start_pose` describes the pose the plan was designed for (empty when not configured); show it before homing. `from_profile` is false when the profile has no `[home_all]` and the plan is the default of one axis per group in profile order.

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

Each value is prefixed with the enum name in the proto, e.g. `ROBOT_STATE_READY`. A newer engine with the same `0.MINOR` may add values; clients must handle unknown ones.

## Versions

`api_version` is `0.MINOR.PATCH` while MotionCore is pre-1.0. Clients accept an engine with the same `0.MINOR`.

| Version | Change |
|---|---|
| 0.1.0 – 0.1.2 | Initial contract (`go_package`, organisation move) |
| 0.1.3 | `AxisConfig.geometry` (`AxisGeometry`, `Vec3`) |
| 0.1.4 | `HomeAll` RPC, `RobotConfig.home_all` (`HomeAllPlan`, `HomeAllGroup`), `RobotConfig.collision_checking`, `AxisGeometry.radius_mm`, `AxisTelemetry.homing`, error code `collision` |
| 0.2.0 | `MoveJ` RPC (`MoveJRequest`, `JointTarget`): coordinated joint moves. New minor: clients accept `0.2.x` engines only |

- **PATCH:** additive changes only (new fields, messages, RPCs and enum values).
- **MINOR:** breaking changes and major features; coordinated MoveJ will be `0.2.0`.
- Field numbers are never reused; removed fields are `reserved`.

Versions 0.1.0–0.1.4 were first published as 1.0.0–1.2.0. See [Contracts & versioning](/internals/versioning#engine-api).

See [Contracts & versioning](/internals/versioning).
