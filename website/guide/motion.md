# Moving axes

`jog` and `move` address one axis; `movej` moves several joints **together** (see [Coordinated joint moves](#coordinated-joint-moves-movej)), `movej-path` runs through several poses without stopping ([Blended paths](#blended-paths-movejpath)) and `movel` moves the tool in a straight line ([Cartesian moves](#cartesian-moves-movel)). A new motion command is only accepted while the robot is `ready`, so wait for the previous one to finish.

The same commands are available from the CLI, the [REST API](https://ratavyr.bzy-logics.com/reference/rest-api) and the [engine gRPC API](/reference/engine-api):

| Action | CLI | REST (`POST /api/commands/{name}`) | gRPC |
|---|---|---|---|
| Energise drives | `motioncore enable` | `enable` | `Enable` |
| De-energise (controlled stop first) | `motioncore disable` | `disable` | `Disable` |
| Home an axis | `motioncore home j1` | `home` `{"axis":"j1"}` | `Home` |
| Home every axis | `motioncore home-all` | `home_all` | `HomeAll` |
| Coordinated joint move | `motioncore movej j1=30 j2=-10 [--speed 0.5]` | `move_j` `{"targets":{"j1":30,"j2":-10}}` | `MoveJ` |
| Blended joint path | `motioncore movej-path "j1=30 j2=10" "j1=-30" [--speed 0.5]` | `move_j_path` `{"waypoints":[{"j1":30,"j2":10},{"j1":-30}]}` | `MoveJPath` |
| Cartesian straight line | `motioncore movel 120 40 300 [--rpy 0,90,0] [--speed 60]` | `move_l` `{"position_mm":[120,40,300]}` | `MoveL` |
| Relative move | `motioncore jog j1 5 [--velocity V]` | `jog` `{"axis":"j1","delta":5}` | `Jog` |
| Absolute move | `motioncore move j1 -90 [--velocity V]` | `move_to` `{"axis":"j1","position":-90}` | `MoveTo` |
| Controlled stop | `motioncore stop` | `stop` | `Stop` |
| Emergency stop | `motioncore estop` | `estop` | `EmergencyStop` |
| Clear a fault | `motioncore reset-fault` | `reset_fault` | `ResetFault` |

All values are in the axis's unit: degrees for revolute axes, millimetres for prismatic ones.

## A typical session

1. **`enable`** → state `ready`.
2. **`home`** each axis that has homing configured, or **`home-all`**. The axis searches for its endstop at the homing speed and is assigned the configured home position. `home-all` follows the profile's [home-all plan](/guide/configuration#home-all-plan): axes in one group home together.
3. **`move`** or **`jog`**. Each returns as soon as the engine accepts it; watch telemetry (`motioncore watch`) for progress. The state is `running` while the axis moves and returns to `ready`.
4. **`stop`** to interrupt a move with controlled deceleration, or **`disable`** to stop and de-energise.

## Coordinated joint moves (MoveJ)

`movej` moves any set of joints to absolute targets so that they **start and finish at the same time**. Joints not listed hold their position.

```bash
motioncore movej j1=60 j2=30 j3=120 j4=90 j5=-60 j6=180 --speed 0.5
```

- **Timing.** Every joint gets a trapezoidal profile with the **same** acceleration, cruise and deceleration times: those of the slowest joint, stretched by the smallest factor that keeps every joint within its `max_velocity` and `max_acceleration`. A joint with less distance to cover simply moves more slowly.
- **Path.** Because the phase times are shared, each joint is always at the same fraction of its own distance. The path is a **straight line in joint space** (not in Cartesian space: the tool does not move in a straight line).
- **Speed.** `--speed` (`speed_scale`, 0 < scale ≤ 1, default 1) scales every joint's velocity limit; acceleration limits are unchanged.
- **Rules.** Every listed joint must be homed and its target within the soft limits; each joint may be listed once. With [collision checking](/guide/configuration#collision-checking), the whole path is checked before anything moves and refused with `collision` if links would come within 2 mm; the clearance is certified between the checked poses too.
- **Execution.** The engine converts every joint's profile into MCU segments on one time grid, so segments of all joints start and end at the same MCU timestamps, and feeds them in time order. The MCU runs every axis from the same clock tick. In simulation all six joints of the desktop arm stay at the same fraction of their move to within one step.

![The six-axis arm in the middle of a coordinated move: j1, j2 and j5 each 45 % of the way](/screenshots/dummy-arm-movej.png)

## Blended paths (MoveJPath)

`movej-path` moves through a list of joint poses **without stopping** at the intermediate ones, starting and ending at rest. Each argument is one waypoint; joints not listed keep their value from the previous waypoint.

```bash
motioncore movej-path "j1=0 j2=0 j3=90 j4=0 j5=0 j6=0" "j1=60 j2=30 j3=120 j4=90 j5=-60 j6=180" "j1=0 j2=45 j3=60 j4=0 j5=-75 j6=0"
```

- **Path.** The waypoints are joined by straight joint-space segments, travelled at constant velocity. Every corner is **rounded** by a constant-acceleration blend shared by all joints, so the robot passes *near* intermediate waypoints (the blend cuts the corner) and exactly through the first and last.
- **Timing.** Each segment starts at the time-optimal duration for its slowest joint; blends last as long as the largest velocity change needs at the joints' `max_acceleration`. Segments are stretched until every blend fits. The robot slows down only as much as a corner requires: a corner that doubles back on itself passes through zero velocity.
- **Rules.** As for `movej`: every joint used must be homed, every waypoint within the soft limits (errors name the waypoint), at most 256 waypoints. With collision checking, the whole rounded path is checked before anything moves (as straight lines 5 ms apart, widened by how far the curve can bow away from them).
- **Execution.** Every piece has constant acceleration, so the MCU segments are exact, as for single moves.

## Cartesian moves (MoveL)

`movel` moves the **tool point** (the tip of the last joint) in a straight line to a position in the robot base frame, in mm. With six or more joints the orientation is held, or turned about a single axis to `--rpy ROLL,PITCH,YAW` (degrees, `R = Rz(yaw)·Ry(pitch)·Rx(roll)`); arms with fewer joints control the position only.

```bash
motioncore status                          # "tool x … y … z … mm  roll … pitch … yaw … deg"
motioncore movel 89.4 120 106.7 --speed 60
```

- **Requirements.** The profile's [geometry](/guide/configuration#_7-geometry-optional) must be a single serial chain (each joint mounted on the previous one); the tip of the last joint is the tool point. Every axis must be homed. Telemetry then carries the tool pose (`tool`), computed from the measured joint positions.
- **Planning.** The line is sampled every 0.5 mm and 0.25° of tool motion; a numeric inverse-kinematics solver (damped least squares, starting from the previous sample so the arm stays on its branch) gives the joint values. The move is refused when the line is out of reach, would need a joint beyond its soft limits, or passes so close to a singularity that a joint would jump.
- **Speed.** `--speed` is the tool speed along the line (default 50 mm/s, at most 2000), with 500 mm/s² ramps; rotation is limited to 45°/s. Where a joint would exceed its own `max_velocity` or `max_acceleration`, the whole move is slowed down so the line is kept. `--speed-scale` scales the joint velocity limits as for `movej`.
- **Safety.** With collision checking, the joint path through the line's inverse-kinematics samples (every 0.5 mm / 0.25°) is checked before anything moves, with the clearance certified between samples. The MCU receives 10 ms segments sampled from the joint trajectory; position errors do not accumulate (step counts come from absolute positions). In simulation, the measured tool point stays within 0.3 mm of the line.

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
| `bad_request` (400) | `move_j` with no targets or an axis listed twice |
| `collision` (409) | The move would make links collide or reach the floor ([collision checking](/guide/configuration#collision-checking)) |
| `not_homed` (400) | `move` on an unhomed axis |
| `validation_failed` (400) | Target outside soft limits, velocity above maximum, non-finite value |
| `bad_request` (400) | `home` on an axis without `[axis.homing]` |
| `unknown_axis` (404) | Axis id not in the configuration |
| `mcu_error` (502) | MCU rejected the request or the link failed |

Full list: [Faults & error codes](/reference/errors).
