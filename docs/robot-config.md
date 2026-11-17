# Robot configuration — schema 1

TOML, validated by `crates/device-graph`. Unknown keys are errors.
All problems are reported at once with a path such as `axis[0](j1).driver.dir_pin`.

```toml
schema = 1                                   # required, must be 1

[robot]
name = "single-axis-bench"
profile = "org.motioncore.robot.single_axis" # robot-profile plugin id (must be active)

[mcu.<name>]                                 # exactly one MCU in v0.1
board = "virtual" | "stm32f411-blackpill" | "rp2040-pico"   # see `motioncore firmware targets`
transport = "virtual" | "tcp://HOST:PORT" | "serial:///dev/ttyACM0"
watchdog_ms = 500                            # 10..=10000, MCU communication watchdog

[[axis]]
id = "j1"                                    # [A-Za-z0-9_], unique
name = "Base"
kind = "revolute" | "prismatic"
unit = "deg" | "mm"                          # revolute → deg, prismatic → mm
mcu = "<name>"
channel = 0                                  # < board step channels, unique per MCU
steps_per_unit = 44.444444                   # motor steps × microsteps × gearing / unit
min = -170.0                                 # soft limits, axis units
max = 170.0
max_velocity = 90.0                          # units/s; × steps_per_unit ≤ board max step rate
max_acceleration = 360.0                     # units/s²
stop_deceleration = 720.0                    # optional, default 2 × max_acceleration
invert_dir = false                           # optional

[axis.driver]
actuator = "org.motioncore.actuator.step_dir" # actuator plugin id (must be active)
step_pin = "PA0"                             # must exist on the board, unique per MCU
dir_pin = "PA1"
enable_pin = "PA2"

[axis.endstop]                               # optional
pin = "PB0"
invert = false

[axis.homing]                                # optional, requires [axis.endstop]
direction = "negative" | "positive"
speed = 30.0                                 # units/s, ≤ max_velocity
position = -170.0                            # assigned at the switch, within [min, max]
max_travel = 360.0                           # fail homing after this much travel
```

## Geometry (optional)

`[axis.geometry]` places each joint in the robot's kinematic tree, so clients
can draw the real machine (and, later, compute tool positions). It follows
the URDF joint convention:

```text
frame(child) = frame(parent) · Translate(origin) · RPY(rotation) · Joint(q about axis)
```

`frame(parent)` already includes the parent joint's motion; an axis without
`parent` is mounted on the robot base (world frame).

```toml
[axis.geometry]
parent = "j1"                 # axis this joint is mounted on; omit for the base
origin = [250.0, 0.0, 0.0]    # joint origin in the parent frame, mm (default 0)
rotation = [0.0, 0.0, 0.0]    # fixed roll, pitch, yaw in deg (about X, Y, Z, in that order; default 0)
axis = [0.0, 0.0, 1.0]        # rotation axis (revolute) or slide direction (prismatic); default +Z
tip = [200.0, 0.0, 0.0]       # optional: end of this joint's link (e.g. tool flange), in its joint frame, mm
```

A link is drawn from each joint to the origins of the joints mounted on it.
`tip` gives the last joint of a chain (nothing mounted on it) its visible
length; without it only the joint itself is drawn.

Rules checked by the resolver:

* All or nothing: if one axis has geometry, every axis must.
* `parent` must name another axis; chains may not form a cycle.
* All numbers finite; `axis` must be non-zero (it is normalised).

Geometry only describes the machine for display and future kinematics.
Motion is still commanded per axis; it does not enable coordinated
multi-axis moves. See `robot-profiles/rail-arm-virtual.toml` for a complete
example: a two-link arm on a linear rail.
