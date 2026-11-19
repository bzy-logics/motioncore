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
radius = 20.0                 # optional: link thickness, mm; enables collision checking (see below)
```

A link is drawn from each joint to the origins of the joints mounted on it.
`tip` gives the last joint of a chain (nothing mounted on it) its visible
length; without it only the joint itself is drawn.

Rules checked by the resolver:

* All or nothing: if one axis has geometry, every axis must.
* `parent` must name another axis; chains may not form a cycle.
* All numbers finite; `axis` must be non-zero (it is normalised).
* `radius`, when given, is positive, and all or nothing: every axis or none.

Geometry is not needed for coordinated joint moves (`MoveJ`), which are
planned in joint space; it is used for display and collision checking. See `robot-profiles/rail-arm-virtual.toml` (a two-link arm
on a linear rail) and `robot-profiles/dummy-arm-virtual.toml` (a six-axis
arm with link radii and a home-all plan) for complete examples.

### Collision checking

With a `radius` on every axis, the engine models each link as capsules of
that radius: from the joint to the joints mounted on it, and to its `tip`.
Once **every** axis is homed, it checks every move before starting it
(`move_to`, `jog`, `move_j`, `move_j_path`, `move_l`) and refuses it with
error `collision` when two links would come closer than 2 mm, or a link
(other than one standing on the base) would reach the floor (z = 0).

The check covers the whole planned path, not only the poses it evaluates.
From the geometry, the engine bounds how fast each joint can move any point
of each link (its distance from the joint's axis). From each checked pose it
steps only as far as the remaining clearance above 2 mm allows, so no contact
can hide between two checked poses: long steps far from other links, short
ones near them. The planned path is the straight joint-space line of
`move_to`, `jog` and `move_j`, the polyline through the inverse-kinematics
samples of `move_l`, and the blended curve of `move_j_path` (checked as lines
5 ms apart, widened by how far the curve can bow away from them). A path that
cannot be shown to keep 2 mm within 0.01 mm is refused. The log line of an
accepted move reports the certified clearance (`clearance >= 2.7 mm`).

What this guarantees and what it does not:

- It is a guarantee about the capsule model, not the machine: rigid links,
  the profile's radii, nothing else in the cell. Choose radii that enclose
  the real links, motors and cables, with room to spare.
- It covers the planned joint path. The motors follow it within the MCU's
  step tolerance (±1 step in simulation; not yet measured on hardware), and
  `move_l` runs as 10 ms segments between path samples; these tracking errors
  are a few hundredths of a millimetre on the desktop arm but are not part of
  the guarantee.
- It is a pre-move check, not a runtime monitor, and no substitute for
  guarding or a hardware E-stop.

A robot already closer than 2 mm (for example after a geometry change) may
back out: a move is allowed if no gap that started below 2 mm dips more than
0.1 mm below its start or ends smaller than it started.

Links that touch by construction are not compared: a joint and the joint
mounted on it, and joints connected only through zero-length links (such as
the three joints of a spherical wrist). The model is the profile's
description, not a measurement of the machine: choose radii that enclose the
real links, motors and cables. Before homing, joint positions are unknown and
nothing is checked; see the home-all plan below.

## Home-all plan (optional)

`home_all` homes every axis that has `[axis.homing]`. Without a plan, axes
home one at a time in profile order. A plan lets several axes home at the
same time:

```toml
[home_all]
groups = [["j4", "j5", "j6"], ["j1", "j2", "j3"]]   # axes of a group home together; groups run in order
start_pose = "Arm straight up with every joint at its zero mark."  # optional, shown before homing
```

* Every group names known axes with `[axis.homing]`; no axis appears twice,
  and every homeable axis is in exactly one group.
* The next group starts only when every axis of the current one reached its
  switch. A fault, Stop or E-stop ends the plan.

Joint positions are unknown until each axis reaches its switch, so the
engine cannot check homing paths for collisions. Choose the groups for a
known start pose (simulate or measure the paths from it) and describe that
pose in `start_pose`; clients show it before homing starts.
