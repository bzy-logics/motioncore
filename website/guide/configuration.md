# Robot configuration

A robot is described by one TOML file: its MCU, its axes, and how each axis is wired. The engine loads it at start (`motioncore start --config FILE`), and the firmware builder uses the same file to generate firmware.

For every key and its constraints, see the [configuration schema reference](/reference/config-schema). This page walks through writing one.

## A minimal profile

This is the shipped `robot-profiles/single-axis-virtual.toml`: one revolute axis on the virtual MCU.

```toml
schema = 1

[robot]
name = "single-axis-bench"
profile = "org.motioncore.robot.single_axis"

[mcu.main]
board = "virtual"
transport = "virtual"
watchdog_ms = 500

[[axis]]
id = "j1"
name = "Base"
kind = "revolute"
unit = "deg"
mcu = "main"
channel = 0
# 200 steps/rev * 8 microsteps * 10:1 gearbox / 360 deg
steps_per_unit = 44.444444
min = -170.0
max = 170.0
max_velocity = 90.0
max_acceleration = 360.0

[axis.driver]
actuator = "org.motioncore.actuator.step_dir"
step_pin = "STEP0"
dir_pin = "DIR0"
enable_pin = "EN0"

[axis.endstop]
pin = "ENDSTOP0"

[axis.homing]
direction = "negative"
speed = 30.0
position = -170.0
max_travel = 360.0
```

## Step by step

### 1. Robot and profile plugin

`robot.profile` names a robot-profile plugin, which must be active. For now, use the builtin `org.motioncore.robot.single_axis`. Each axis's `driver.actuator` names an actuator plugin; use `org.motioncore.actuator.step_dir`. See [Plugins](/guide/plugins).

### 2. The MCU

Pick a `board` and a `transport`:

| board | transport examples |
|---|---|
| `virtual` | `"virtual"` (in-process) or `"tcp://127.0.0.1:7700"` (out-of-process [simulator](/guide/quick-start#_4-try-a-fault-unplug-the-simulator)) |
| `stm32f411-blackpill` | `"serial:///dev/ttyACM0"` |
| `rp2040-pico` | `"serial:///dev/ttyACM0"` (`COMx` on Windows) |

`watchdog_ms` is how long the MCU waits without hearing from the host before it disables every drive. The default of 500 ms is a good start.

v0.2 supports **exactly one MCU** per robot.

### 3. Axes: units and steps

Each `[[axis]]` is one motor. `kind = "revolute"` uses degrees; `kind = "prismatic"` uses millimetres. All positions, velocities and accelerations are in that unit.

`steps_per_unit` converts units to motor steps:

```text
steps_per_unit = full steps per rev × microsteps × gear ratio / units per output rev
```

| Example | Calculation | Result |
|---|---|---|
| Revolute joint, 1.8° motor, 8× microstepping, 10:1 gearbox | 200 × 8 × 10 / 360 | 44.444444 steps/° |
| Lead screw, 1.8° motor, 16× microstepping, 8 mm lead | 200 × 16 / 8 | 400 steps/mm |

### 4. Limits

- `min` / `max` are soft limits, enforced by both the planner and the MCU.
- `max_velocity × steps_per_unit` must not exceed the board's maximum step rate (`motioncore firmware targets` lists it). For example, on RP2040 (50 000 steps/s advertised), 44.44 steps/° allows up to 1125 °/s.
- `stop_deceleration` defaults to twice `max_acceleration`. It is used by `stop` and by a controlled stop on `disable`.

### 5. Wiring

`channel` selects the MCU's step output, and the pin names must match **the board's fixed firmware pin map**. The resolver rejects a mismatch with a specific message, for example:

```text
board 'rp2040-pico' firmware wires channel 1 driver.step_pin to GP2
```

Pin maps: [STM32F411](/hardware/stm32f411#wiring), [RP2040](/hardware/rp2040#pin-map). The virtual board has no fixed pin map, so any names work there; the shipped profiles use `STEPn`, `DIRn`, `ENn`, `ENDSTOPn` by convention. Pins must still be unique.

### 6. Endstop and homing (optional, recommended)

Add `[axis.endstop]` with the switch's pin, and `invert = true` for a normally-closed switch. With an endstop, `[axis.homing]` tells the axis to search in `direction` at `speed` until the switch triggers, then to set its position to `position`. Homing fails if the switch is not found within `max_travel`.

Without homing, an axis can only be jogged relatively, at 25 % speed. See [Moving axes](/guide/motion).

### 7. Geometry (optional)

`[axis.geometry]` places each joint in the robot's kinematic tree, so clients such as [sindrava's 3D viewer](https://sindrava.bzy-logics.com/guide/screens#_3d-viewer) can draw the real, connected machine. It follows the URDF joint convention:

```text
frame(child) = frame(parent) · Translate(origin) · RPY(rotation) · Joint(q about axis)
```

`frame(parent)` already includes the parent joint's motion; an axis without `parent` is mounted on the robot base.

```toml
[axis.geometry]
parent = "j1"                 # axis this joint is mounted on; omit for the robot base
origin = [250.0, 0.0, 0.0]    # joint origin in the parent frame, mm (default 0)
rotation = [0.0, 0.0, 0.0]    # fixed roll, pitch, yaw in degrees (about X, Y, Z, in that order)
axis = [0.0, 0.0, 1.0]        # rotation axis (revolute) or slide direction (prismatic); default +Z
tip = [200.0, 0.0, 0.0]       # optional: end of this joint's link (e.g. tool flange), mm
```

Links are drawn from each joint to the joints mounted on it; `tip` gives the last joint of a chain its length. If one axis has geometry, every axis needs it. Geometry describes the machine for display (and future kinematics) only: motion is still commanded one axis at a time. Requires engine API 1.1.

![A two-link arm on a linear rail, drawn from its geometry in sindrava](/screenshots/rail-arm-viewer.png)

## Validation

The configuration is validated by the device graph when the engine starts and when you build firmware:

- `schema` must be `1`; **unknown keys are errors** (so typos don't silently fall back to defaults).
- **All problems are reported at once**, each with a path such as `axis[0](j1).driver.dir_pin`.
- Axis ids must be unique; channels and pins must be unique per MCU; channels must exist on the board.
- Pins must match the board's firmware pin map.
- Homing requires an endstop, a speed ≤ `max_velocity`, and a `position` within `[min, max]`.
- Geometry is all-or-nothing; parents must exist and may not form a cycle; joint axes must be non-zero.

To check a file without starting the engine, run the firmware configure step, which resolves the configuration and prints the lockfile:

```bash
motioncore firmware configure --config my-robot.toml
```

## Shipped profiles

| File | Board | Axes |
|---|---|---|
| `robot-profiles/single-axis-virtual.toml` | `virtual` | 1 |
| `robot-profiles/rail-arm-virtual.toml` | `virtual` | 3: a two-link arm on a linear rail, with geometry |
| `robot-profiles/single-axis-stm32f411.toml` | `stm32f411-blackpill` | 1 |
| `robot-profiles/six-axis-rp2040-pico.toml` | `rp2040-pico` | 6, with endstops and homing |

## Changing the configuration

The engine reads the file only at start. There is no configuration-editing API yet: edit the TOML and restart the engine. The UI's Robot Composer is read-only for the same reason. If you change pins, channels or boards, rebuild and reflash the firmware too.
