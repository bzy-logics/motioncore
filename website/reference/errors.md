# Faults & error codes

There are two different things here:

- **Robot faults** are latched conditions that put the robot into `faulted` (or `emergency_stopped`) and disable the drives. They appear in telemetry's `fault` field.
- **Error codes** are returned when a single request is refused. The robot's state doesn't change.

## Robot faults

| Code | Detected by | Cause | Response |
|---|---|---|---|
| `communication_lost` | MCU watchdog **or** engine | MCU heard nothing from the host for `watchdog_ms` (e.g. USB unplugged, host hung); or the engine heard nothing from the MCU for 1 s, or the stream closed | Drives disabled, queue flushed. The engine attempts to reconnect |
| `queue_underrun` | MCU | The segment queue ran dry in the middle of a move | Drives disabled |
| `segment_late` | MCU | A segment started too late, or the step tick stalled during motion | Drives disabled; no catch-up steps |
| `limit_hit` | MCU | Endstop triggered while moving toward the switch | No further step; drives disabled. Reset is refused while the axis is still on the switch with no known safe direction — move it off by hand |
| `soft_limit` | engine (planning) and MCU (per step) | Motion would leave `[min, max]` | Request rejected by the engine; if the MCU catches it, drives disabled |
| `homing_failed` | MCU | No endstop trigger within `max_travel` | Drives disabled |
| `mcu_fault` | MCU | Other firmware fault, e.g. `StepRateExceeded` on RP2040 when PIO output falls behind or a window's steps don't fit the pulse timing | Drives disabled |
| `plugin_failed` | plugin runtime | A plugin the robot requires failed | Plugin and its dependents marked `failed` |
| `emergency_stop` | engine + MCU | Software E-stop, or on RP2040 the GP27 interlock opened | Latched `emergency_stopped`, drives disabled |

### Clearing a fault

1. Remove the cause: reconnect the cable, close the interlock, move off the switch by hand with drives off, and so on.
2. `reset-fault` (CLI), `POST /api/commands/reset_fault`, or `ResetFault` (gRPC).
3. The robot lands in `disabled` (or `disconnected` if the MCU link is down). It **never** resumes the interrupted motion.
4. `enable` and **re-home every axis**; faults clear the homed flag.

Every fault latched by the MCU disables **all** drives. Only the first fault is recorded; later faults don't overwrite it.

The MCU refuses a reset until the hazard is verifiably gone: it is refused while any drive is enabled or moving, while an endstop that caused `limit_hit` is still pressed, and on RP2040 while the GP27 interlock is open.

## HTTP error codes

Through the ratavyr gateway, these codes are returned as HTTP statuses with a JSON error envelope. See [ratavyr: error codes](https://ratavyr.bzy-logics.com/reference/errors).

## gRPC error codes

The engine returns gRPC status codes, with the precise code in the trailer `motioncore-error-code`:

| Code | gRPC status |
|---|---|
| `bad_request`, `validation_failed` | `INVALID_ARGUMENT` |
| `unknown_axis`, `not_found` | `NOT_FOUND` |
| `not_homed`, `invalid_state` | `FAILED_PRECONDITION` |
| `mcu_error` | `UNAVAILABLE` |
| `internal` | `INTERNAL` |

## MCU NACK reasons

Inside the [MCU protocol](/internals/mcu-protocol), the firmware refuses individual frames with a NACK. These surface to clients as `mcu_error` or as a fault.

| Reason | Meaning |
|---|---|
| `UnknownAxis` | Channel not configured |
| `QueueFull` | Segment queue full |
| `InvalidState` | Not allowed in the MCU's current state |
| `InvalidParams` | Parameters failed validation |
| `SoftLimit` | Would leave the soft limits |
| `Faulted` | A fault is latched |
| `Busy` | Axis is moving |
