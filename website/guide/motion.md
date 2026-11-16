# Moving axes

All motion commands address **one axis at a time**. Coordinated multi-axis moves are not available yet. A new motion command is only accepted while the robot is `ready`, so wait for the previous one to finish.

The same commands are available from the CLI, the [REST API](https://ratavyr.bzy-logics.com/reference/rest-api) and the [engine gRPC API](/reference/engine-api):

| Action | CLI | REST (`POST /api/commands/{name}`) | gRPC |
|---|---|---|---|
| Energise drives | `motioncore enable` | `enable` | `Enable` |
| De-energise (controlled stop first) | `motioncore disable` | `disable` | `Disable` |
| Home an axis | `motioncore home j1` | `home` `{"axis":"j1"}` | `Home` |
| Relative move | `motioncore jog j1 5 [--velocity V]` | `jog` `{"axis":"j1","delta":5}` | `Jog` |
| Absolute move | `motioncore move j1 -90 [--velocity V]` | `move_to` `{"axis":"j1","position":-90}` | `MoveTo` |
| Controlled stop | `motioncore stop` | `stop` | `Stop` |
| Emergency stop | `motioncore estop` | `estop` | `EmergencyStop` |
| Clear a fault | `motioncore reset-fault` | `reset_fault` | `ResetFault` |

All values are in the axis's unit: degrees for revolute axes, millimetres for prismatic ones.

## A typical session

1. **`enable`** → state `ready`.
2. **`home`** each axis that has homing configured. The axis searches for its endstop at the homing speed and is assigned the configured home position.
3. **`move`** or **`jog`**. Each returns as soon as the engine accepts it; watch telemetry (`motioncore watch`) for progress. The state is `running` while the axis moves and returns to `ready`.
4. **`stop`** to interrupt a move with controlled deceleration, or **`disable`** to stop and de-energise.

## Homed vs. unhomed

| | Unhomed | Homed |
|---|---|---|
| `jog` | Allowed, at **25 %** of `max_velocity` by default, and only within one travel range of the power-on position | Allowed, default velocity `max_velocity` |
| `move` | Rejected (`not_homed`) | Allowed |
| Soft limits | Relative to an unknown origin | Enforced against real positions |

An axis loses its homed flag on **any fault or E-stop**. Re-home after `reset-fault`.

## Velocity

`--velocity` (CLI) or `velocity` (API) sets the cruise velocity. It is optional; the default is the axis maximum.

- **Homed axis:** a velocity above `max_velocity` is rejected with `validation_failed`.
- **Unhomed axis:** the velocity is capped at 25 % of `max_velocity`; a higher request is silently clamped to the cap.

Moves that are too short to reach the cruise velocity simply follow a triangular profile.

## Positions in telemetry

Each axis reports:

| Field | Meaning |
|---|---|
| `commanded_position` | Where the planner has commanded the axis to be |
| `measured_position` | From the MCU's step counter. There is no encoder feedback yet, so this shows what the MCU emitted, not what the motor did |
| `velocity` | Current velocity |
| `homed`, `enabled`, `endstop` | Flags |

When the MCU reports idle, commanded positions are re-synced from the MCU step counter.

## When a command is refused

| Error | Typical cause |
|---|---|
| `invalid_state` (409) | Robot not `ready`: still moving, disabled, faulted or E-stopped |
| `not_homed` (400) | `move` on an unhomed axis |
| `validation_failed` (400) | Target outside soft limits, velocity above maximum, non-finite value |
| `bad_request` (400) | `home` on an axis without `[axis.homing]` |
| `unknown_axis` (404) | Axis id not in the configuration |
| `mcu_error` (502) | MCU rejected the request or the link failed |

Full list: [Faults & error codes](/reference/errors).
